import sys
import time
from pymavlink import mavutil

# Ensure the user provides the UAV ID when running the script
if len(sys.argv) != 2:
    print("Usage: sudo ip netns exec uavX python3 cluster_logic.py <UAV_ID>")
    print("Example: sudo ip netns exec uav1 python3 cluster_logic.py 1")
    sys.exit(1)

my_id = int(sys.argv[1])

print(f"--- UAV {my_id} Cluster Logic Booting ---")

# 1. SETUP MAVLINK UDP CONNECTIONS
# Listen for incoming messages on port 14550 from any IP
try:
    incoming = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Broadcast outgoing messages to the entire 10.42.x.x subnet
    outgoing = mavutil.mavlink_connection('udpout:10.42.255.255:14550', source_system=my_id)
except Exception as e:
    print(f"Network bind error: {e}")
    sys.exit(1)

# 2. ELECTION VARIABLES
link_scores = {}          # Stores the latest scores from other drones
my_link_cost = float(my_id) # Baseline cost (In reality, calculate this from packet drops)
last_ping_time = time.time()
last_election_time = time.time()

print(f"UAV {my_id} listening to swarm on 10.42.255.255:14550...")

# 3. THE MAIN LOOP
while True:
    current_time = time.time()

    # --- STEP A: READ INCOMING NETWORK TRAFFIC ---
    # Non-blocking read to check if any messages arrived from NS-3
    msg = incoming.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        text = msg.text
        sender_id = msg.get_srcSystem()
        
        # Ignore our own broadcast echoes
        if sender_id != my_id:
            parts = text.split(':')
            
            # If someone PINGs us, we could track it here
            if parts[0] == "PING":
                pass # (Advanced logic: Send ACK back)
            
            # If someone broadcasts their SCORE, update our election table
            elif parts[0] == "SCORE":
                other_cost = float(parts[1])
                link_scores[sender_id] = other_cost

    # --- STEP B: SEND PINGS (Every 1 second) ---
    if current_time - last_ping_time > 1.0:
        ping_msg = f"PING:{my_id}"
        # Send the message into the namespace's network
        outgoing.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, ping_msg.encode())
        last_ping_time = current_time

    # --- STEP C: ELECTION & BROADCAST SCORE (Every 5 seconds) ---
    if current_time - last_election_time > 5.0:
        # 1. Broadcast our current score to the swarm
        score_msg = f"SCORE:{my_link_cost}"
        outgoing.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, score_msg.encode())
        
        # 2. Run the Election Algorithm
        best_id = my_id
        best_cost = my_link_cost
        
        # Compare our score against everyone else's
        for uav, cost in link_scores.items():
            if cost < best_cost:
                best_cost = cost
                best_id = uav
                
        # Declare the results
        if best_id == my_id:
            print(f"[{time.strftime('%X')}] I am the Cluster Head! (Lowest Score: {my_link_cost})")
        else:
            print(f"[{time.strftime('%X')}] UAV {best_id} is the Head. (My Score: {my_link_cost})")
            
        last_election_time = current_time
        
    # Small sleep to prevent the while loop from pegging your CPU at 100%
    time.sleep(0.01)
