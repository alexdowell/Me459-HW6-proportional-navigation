# ME459 HW6: Proportional Navigation and Evasion Strategies  

## Description  
This repository contains implementations of **proportional navigation (PN) and evasion strategies** for **ME459 Robotics and Unmanned Systems**. The focus is on developing **pursuer-evader models** using **Turtlebot simulations** to test pursuit and evasion behaviors under various conditions. The repository includes Python scripts for implementing **proportional navigation, trajectory planning, and evasion algorithms**, along with datasets for evaluating different strategies.  

## Files Included  

### **Proportional Navigation Pursuit Algorithm**  
- **File:** prop_nav.py  
- **Topics Covered:**  
  - Proportional navigation for target pursuit  
  - Lidar-based target tracking  
  - Heading and turn rate calculations  
  - ROS integration for real-time tracking  

### **Evasion Strategy Implementation**  
- **File:** umkc_evader.py  
- **Topics Covered:**  
  - Randomized waypoint evasion  
  - Reactive and planned evasion techniques  
  - Simulation-based pursuit testing  
  - Turtlebot control in ROS  

### **Turtlebot Control Script**  
- **File:** Turtlebot.py  
- **Topics Covered:**  
  - PID-based movement control  
  - Obstacle avoidance with Lidar  
  - Speed and trajectory adjustments  
  - ROS-based bot communication  

### **Performance Data and Evaluation**  
- **File:** ME 459 HW 6.pdf  
  - Description of pursuit and evasion experiments  
  - Proportional navigation parameter tuning  
  - Success and failure case studies  

- **Files:** ME 459 HW 6 pr 3 trial 1.csv, ME 459 HW 6 pr 3 trial 2.csv, ME 459 HW 6 pr 3 trial 3.csv  
  - Data from randomized evasion trials  
  - Position and time logs for evaluation  
  
- **File:** ME 459 HW 6 pr 4.csv  
  - Dataset for evasion scenario with defined speed constraints  

- **Files:** ME 459 HW 6 pr 5 evading.csv, ME 459 HW 6 pr 5 pursuing.csv  
  - Data from adversarial evader-pursuer matches  
  - Performance comparison of different evasion and pursuit strategies  

## Installation  
Ensure Python and the required libraries are installed before running the scripts.  

### **Required Python Packages**  
- numpy  
- matplotlib  
- rospy  
- geometry_msgs  
- sensor_msgs  
- nav_msgs  

To install the necessary packages, run:  

```pip install numpy matplotlib rospy geometry_msgs sensor_msgs nav_msgs```

## Usage  
1. Open a terminal and launch the ROS environment.  
2. Run the desired script using:  

```python prop_nav.py```  
```python umkc_evader.py```  
```python Turtlebot.py```  

3. Analyze simulation results by plotting the output data.  

## Example Output  

### **Proportional Navigation Pursuit**  
- Successfully intercepts evading target within 30 seconds  
- Adjusts turn rate dynamically based on LOS angle  

### **Evasion Strategy**  
- Tests random and pre-planned evasive maneuvers  
- Performance evaluated based on evasion success rates  

## Contributions  
This repository is designed for educational and research purposes. Feel free to modify and expand upon the existing implementations.  

## License  
This project is open for educational and research use.  

---  

**Author:** Alexander Dowell  

