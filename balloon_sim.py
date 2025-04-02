import serial
import time
import math
import threading
import keyboard
import pandas as pd

# Configuration
COM_PORT = "COM10"
BAUD_RATE = 4800
UPDATE_INTERVAL = 1  # Seconds between GPS updates
EARTH_RADIUS_KM = 6371  # Earth radius for movement calculations

# Initial balloon position (latitude, longitude, altitude in feet)
balloon_lat = 57.89579925190799
balloon_lon = 11.745555042076292
balloon_alt_ft = 106  # Start altitude in feet
vertical_speed_mps = 0  # Initial vertical speed (meters per second)

# Load wind data
winds_df = pd.read_csv("winds.txt", names=["height_ft", "bearing_deg", "speed_knots"])

def calculate_checksum(nmea_sentence):
    """ Calculate the NMEA checksum (XOR of all characters between $ and *). """
    checksum = 0
    for char in nmea_sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def interpolate_wind(altitude_ft):
    """ Interpolates wind speed and bearing correctly, accounting for circular wrap-around """
    sorted_winds = winds_df.sort_values(by="height_ft")  # Ensure heights are sorted
    
    # Find exact match
    exact_match = sorted_winds[sorted_winds["height_ft"] == altitude_ft]
    if not exact_match.empty:
        return exact_match["bearing_deg"].values[0], exact_match["speed_knots"].values[0]

    # Find closest values below and above current altitude
    lower_wind = sorted_winds[sorted_winds["height_ft"] < altitude_ft].iloc[-1]  # Last match below altitude
    upper_wind = sorted_winds[sorted_winds["height_ft"] > altitude_ft].iloc[0]  # First match above altitude

    if lower_wind.empty:  # If below lowest altitude, use lowest
        return upper_wind["bearing_deg"], upper_wind["speed_knots"]
    if upper_wind.empty:  # If above highest altitude, use highest
        return lower_wind["bearing_deg"], lower_wind["speed_knots"]

    # **Circular Interpolation for Bearing**
    lower_bearing = lower_wind["bearing_deg"]
    upper_bearing = upper_wind["bearing_deg"]

    angle_difference = (upper_bearing - lower_bearing) % 360  # Ensure wrap-around works
    if angle_difference > 180:  # Use the shorter path around the circle
        angle_difference -= 360

    fraction = (altitude_ft - lower_wind["height_ft"]) / (upper_wind["height_ft"] - lower_wind["height_ft"])
    interpolated_bearing = (lower_bearing + fraction * angle_difference) % 360  # Keep within 0-360 range
    interpolated_speed = lower_wind["speed_knots"] + fraction * (upper_wind["speed_knots"] - lower_wind["speed_knots"])
    # print(f"Fraction: {fraction:.2f}, low_ft: {lower_wind["height_ft"]}, low_deg: {lower_wind["bearing_deg"]}, up_ft: {upper_wind["height_ft"]}, up_deg: {upper_wind["bearing_deg"]}")

    return interpolated_bearing, interpolated_speed



def update_position(lat, lon, bearing, speed_knots):
    """ Calculates new GPS position based on wind direction and speed, ensuring correct bearing transitions """
    speed_kmh = speed_knots * 1.852  # Convert knots to km/h
    distance_km = speed_kmh * (UPDATE_INTERVAL / 3600)  # Distance traveled

    if distance_km <= 0:  # Prevent invalid calculations
        return lat, lon

    # Normalize bearing to ensure smooth transition (e.g., 359° to 10° doesn't suddenly jump backward)
    normalized_bearing = (bearing + 180) % 360  # Reverse wind direction to get movement direction

    lat_radians = math.radians(lat)
    lon_radians = math.radians(lon)
    bearing_radians = math.radians(normalized_bearing)  # Use the corrected bearing

    try:
        new_lat = math.asin(
            math.sin(lat_radians) * math.cos(distance_km / EARTH_RADIUS_KM) +
            math.cos(lat_radians) * math.sin(distance_km / EARTH_RADIUS_KM) * math.cos(bearing_radians)
        )
        
        new_lon = lon_radians + math.atan2(
            math.sin(bearing_radians) * math.sin(distance_km / EARTH_RADIUS_KM) * math.cos(lat_radians),
            math.cos(distance_km / EARTH_RADIUS_KM) - math.sin(lat_radians) * math.sin(new_lat)
        )

        # Catch any invalid results to prevent NaN errors
        if math.isnan(new_lat) or math.isnan(new_lon):
            print("Warning: Invalid GPS position calculated, keeping previous position.")
            return lat, lon

        return math.degrees(new_lat), math.degrees(new_lon)

    except ValueError as e:
        print(f"Error in position calculation: {e}")
        return lat, lon  # Keep previous position if error occurs

def create_nmea_sentences(lat, lon, alt, speed_knots):
    #Alt should be sent in meters by gps/nmea
    alt = alt / 3.28084

    lat_deg = int(abs(lat))
    lat_min = (abs(lat) - lat_deg) * 60
    lat_hemisphere = "N" if lat >= 0 else "S"

    lon_deg = int(abs(lon))
    lon_min = (abs(lon) - lon_deg) * 60
    lon_hemisphere = "E" if lon >= 0 else "W"

    time_str = time.strftime("%H%M%S.6", time.gmtime())

    rmc_sentence = f"GNRMC,{time_str},A,{lat_deg:02d}{lat_min:.6f},{lat_hemisphere},{lon_deg:03d}{lon_min:.6f},{lon_hemisphere},{speed_knots:.2f},,,002.7,E,N"
    rmc_sentence = f"${rmc_sentence}*{calculate_checksum(rmc_sentence)}"

    gga_sentence = f"GNGGA,{time_str},{lat_deg:02d}{lat_min:.6f},{lat_hemisphere},{lon_deg:03d}{lon_min:.6f},{lon_hemisphere},1,08,0.9,{alt:.1f},M,0.0,M,,"
    gga_sentence = f"${gga_sentence}*{calculate_checksum(gga_sentence)}"

    vtg_sentence = f"GNVTG,,T,,M,{speed_knots:.2f},N,,K,N"
    vtg_sentence = f"${vtg_sentence}*{calculate_checksum(vtg_sentence)}"

    return [rmc_sentence, gga_sentence, vtg_sentence]

# Function to handle real-time keyboard input
def monitor_keyboard():
    global vertical_speed_mps
    while True:
        if keyboard.is_pressed("ä"):
            vertical_speed_mps += 0.1
        elif keyboard.is_pressed("-"):
            vertical_speed_mps -= 0.1
        time.sleep(0.1)  # Frequent input checking

# Start keyboard input monitoring in a separate thread
keyboard_thread = threading.Thread(target=monitor_keyboard, daemon=True)
keyboard_thread.start()

try:
    gps_serial = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"GPS emulator started on {COM_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Waiting for navigation software to connect...")
time.sleep(5)

while True:
    # Adjust altitude based on vertical speed
    balloon_alt_ft += vertical_speed_mps * UPDATE_INTERVAL * 3.281  # Convert m/s to feet

    # Get interpolated wind data
    wind_bearing, wind_speed_knots = interpolate_wind(balloon_alt_ft)

    # Update position based on wind
    # print(f"Wind Bearing: {wind_bearing}, Wind Speed: {wind_speed_knots}")
    # print(f"Altitude: {balloon_alt_ft} ft")
    balloon_lat, balloon_lon = update_position(balloon_lat, balloon_lon, wind_bearing, wind_speed_knots)

    # Create and send NMEA sentences
    nmea_sentences = create_nmea_sentences(balloon_lat, balloon_lon, balloon_alt_ft, wind_speed_knots)
    for sentence in nmea_sentences:
        gps_serial.write((sentence + "\r\n").encode())
        # print(f"Sent: {sentence}")

    # Display flight data
    print(f"Lat: {balloon_lat:.6f}, Lon: {balloon_lon:.6f}, Alt: {balloon_alt_ft:.1f} ft, Speed: {wind_speed_knots:.2f} knots, Vertical Speed: {vertical_speed_mps:.1f} m/s")

    time.sleep(UPDATE_INTERVAL)
