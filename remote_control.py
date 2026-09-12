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

def move_spherical(radius, azimuth, polar, elbow=0, wait=True):
    payload = {"radius": radius, "azimuth": azimuth, "polar": polar, "elbow": elbow}
    res = requests.post(f"{ROBOT_IP}/move/spherical", json=payload, timeout=2)
    print(res.json())
    
    if wait:
        time.sleep(0.1) # brief pause to let movement register
        wait_until_idle()

def move_angles(x, y, z, a, wait=True):
    payload = {"x": x, "y": y, "z": z, "a": a}
    res = requests.post(f"{ROBOT_IP}/move/motors", json=payload, timeout=2)
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

def get_steps():
    url = f"{ROBOT_IP}/position/steps"
    try:
        res = requests.get(url, timeout=2)
        steps = res.json()
        print(f"Current Steps -> X: {steps['x']}, Y: {steps['y']}, Z: {steps['z']}, A: {steps['a']}")
        return steps
    except Exception as e:
        print(f"Error fetching steps: {e}")
        return None

def get_angles():
    url = f"{ROBOT_IP}/position/angles"
    try:
        res = requests.get(url, timeout=2)
        steps = res.json()
        print(f"Current Angles -> X: {steps['x']:.2f}, Y: {steps['y']:.2f}, Z: {steps['z']:.2f} mm, A: {steps['a']:.2f}")
        return steps
    except Exception as e:
        print(f"Error fetching angles: {e}")
        return None

def get_cylinder():
    url = f"{ROBOT_IP}/position/cylinder"
    try:
        res = requests.get(url, timeout=2)
        steps = res.json()
        print(f"Current cylinder coords -> Height: {steps['height']:.2f}, Reach: {steps['reach']:.2f}, Theta: {steps['theta']:.2f}")
        return steps
    except Exception as e:
        print(f"Error fetching cylinder coords: {e}")
        return None

# Example Usage:
#move_steps(x=-5000, y=9720, z=30100, a=13804)
#move_cylindrical(z=15, r=150, theta=0, elbow=0)
#move_cylindrical(z=150, r=200, theta=-40, elbow=0)
#move_spherical(radius=230, azimuth=-50, polar=45, elbow=0)
move_angles(0, 70, 160, 140)

get_steps()
get_angles()
get_cylinder()