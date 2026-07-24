
---
If you are using VPN, please use  Global VPN, otherwise you may Failed to Fetch the files.

### ROS GPG Key Problem

You've hit a common issue with `apt` when dealing with external repositories like ROS: **expired GPG keys**.

The error message: `W: GPG error: http://packages.ros.org/ros/ubuntu focal InRelease: The following signatures were invalid: EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>` `E: The repository 'http://packages.ros.org/ros/ubuntu focal InRelease' is not signed.`

This means that the digital signature used to verify the authenticity of the ROS packages has expired. `apt` (the package manager) is designed to prevent you from installing software from sources it can't verify, as this could be a security risk.

**To resolve this, you need to update the GPG key for the ROS repository.**

---

Many Docker images, especially base ones, are designed to be as small as possible and don't include utilities like `curl` unless you explicitly install them. If you already has `curl`, you can skip that step.

1. **Enter your Docker container:**
    
    ```
    docker exec -it test2 bash
    ```
	
	- remember **replace 'test2' with your container name**
	
2. **Delete the old GPG key 
    
    ```
    sudo apt-key del "F42ED6FBAB17C654"
    ```
    
    You should see `OK`.
    
3. **Install `curl`:**

    ```
    sudo apt update
    sudo apt install -y curl
    ```
    
    - `sudo apt update`: Refreshes your package lists so `apt` knows where to find `curl`.
        
    - `sudo apt install -y curl`: Installs the `curl` package. The `-y` flag automatically confirms the installation, so you don't have to type 'y' when prompted.
        
4. **Download and add the new ROS GPG key (this step will now work):**
    
    ```
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    
    This time, `curl` should execute successfully and download the key.
    
5. **Update your `sources.list.d` entry to use the new key path:**
    

    ```
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list > /dev/null
    ```
    
6. **Update your package list again:**
    
    ```
    sudo apt update
    ```
    
    This should now complete without GPG errors.
    
7. **Install `ros-noetic-ros-tutorials`:**
    
    ```
    sudo apt install ros-noetic-ros-tutorials
    ```
    

