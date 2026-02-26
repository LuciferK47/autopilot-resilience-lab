from pymavlink import mavutil

def get_log_data(filename, msg_type, fields):
    print(f"Extracting {msg_type} from {filename}...")
    data = {f: [] for f in fields}
    
    try:
        # mavutil detects .bin files and picks the correct reader automatically
        log = mavutil.mavlink_connection(filename)
    except Exception as e:
        print(f"CRITICAL: Could not open log file: {e}")
        return data

    # Iterate through the entire log
    while True:
        # recv_match gets the next message of the specific type
        m = log.recv_match(type=msg_type, blocking=False)
        
        # If m is None, we reached the end of the file
        if m is None:
            break
            
        for f in fields:
            try:
                # Extract the value (e.g., m.Roll)
                val = getattr(m, f)
                data[f].append(val)
            except AttributeError:
                pass # Field missing in this specific packet
                
    print(f"Extraction complete. Found {len(list(data.values())[0])} data points.")
    return data