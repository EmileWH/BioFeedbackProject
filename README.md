This project uses an arduino nano, MPU6050, and 16x2 LCD display to track reps and provide a quantitative measure of effort during a bicep curl. 
The MPU measures the arms orientation and calculcates the pitch angle, by monitoring how the pitch angle changes over time, the system can calculate the angular velocity of the arm
From this the program can determine when a rep has been completed, when a speed threshold is crossed a rep begins and ends when the motion slows near the top of the curl
During the upward phase of the rep, the program quantifies effort based on the inverse of the rotational speed. 
Slower movement indicates higher effort and faster movement indicates lower effort
The values for reps completed and and calculated effort are then shown on an LDC screen, while the serial monitor also records this data
