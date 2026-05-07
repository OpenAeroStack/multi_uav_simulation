#!/bin/bash
# run_clustering.sh - Launches dynamic cluster head election across all UAVs

echo "=== Booting Dynamic Swarm Clustering ==="

# Array to keep track of the background Python processes
CLUSTER_PIDS=()

# This function ensures that when you press Ctrl+C, it cleanly kills all 3 Python scripts
cleanup_clustering() {
    echo ""
    echo "=== Shutting down clustering logic ==="
    if [[ ${#CLUSTER_PIDS[@]} -gt 0 ]]; then
        sudo kill "${CLUSTER_PIDS[@]}" 2>/dev/null || true
    fi
    exit 0
}
# Catch the Ctrl+C signal and run the cleanup function
trap cleanup_clustering SIGINT SIGTERM

# Loop through UAV 1, 2, and 3
for i in 1 2 3; do
    echo "Launching Cluster Logic for UAV $i inside namespace 'uav$i'..."
    
    # Run the Python script inside the namespace, passing the ID ($i), 
    # and send it to the background (&) so the loop can continue
    sudo ip netns exec "uav$i" python3 cluster_logic.py "$i" &
    
    # Save the Process ID so we can kill it later
    CLUSTER_PIDS+=($!)
done

echo "=== All clustering scripts are running. ==="
echo "=== Monitoring Swarm... (Press Ctrl+C to stop) ==="

# The 'wait' command pauses the bash script here, keeping the terminal open 
# so you can watch the print() statements from your Python scripts.
wait
