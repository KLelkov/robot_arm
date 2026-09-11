import requests
import time

ROBOT_IP = "http://192.168.1.101"

def wait_until_idle():
    while True:
        try:
            res = requests.get(f"{ROBOT_IP}/status", timeout=2).json()
            if res.get("status") == "idle":
                return
        except Exception:
            pass
        time.sleep(0.2)

def move_cylindrical(z, r, theta, elbow=0, wait=True):
    payload = {"z": z, "r": r, "theta": theta, "elbow": elbow}
    res = requests.post(f"{ROBOT_IP}/move/cylindrical", json=payload, timeout=2)
    print(res.json())
    
    if wait:
        time.sleep(0.1) # brief pause to let movement register
        wait_until_idle()

def move_steps(x, y, z, a, wait=True):
    url = f"{ROBOT_IP}/move/steps"
    payload = {"x": int(x), "y": int(y), "z": int(z), "a": int(a)}
    
    res = requests.post(url, json=payload, timeout=2)
    print("Move Steps Response:", res.json())
    
    if wait:
        time.sleep(0.1)
        wait_until_idle()

# Example Usage:
move_steps(x=0, y=0, z=0, a=5000)
#move_cylindrical(z=3, r=300, theta=136, elbow=0)
#move_cylindrical(z=50, r=200, theta=0, elbow=0)