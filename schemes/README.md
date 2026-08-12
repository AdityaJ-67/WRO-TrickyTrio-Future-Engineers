# Electrical Components

## System Architecture

In this architectural setup, the Raspberry Pi 5 acts as the high-level "Brain" (handling heavy computational tasks), while the Pico 2 W acts as the low-level "Brainstem" (handling real-time hardware execution). [[1]](https://www.hackster.io/HiwonderRobot/pi-5-powerhouse-building-the-ultimate-ros-2-composite-bot-fd2bc1)

### What the Pi 5 is responsible for

**1. Computer Vision and Image Processing**
- **Camera Module 3 Management:** The Camera Module 3 connects directly to the Pi 5 because processing high-definition live video feeds requires immense CPU power and RAM that the Pico 2 W lacks. [[1]](https://www.pocket-lint.com/how-to-use-raspberry-pi-5-beginner-projects/)
- **Visual Intelligence:** The Pi 5 runs heavy workloads like object detection, line tracking, obstacle recognition, or AprilTag/QR code scanning on the live video stream.

**2. High-Level AI and Navigation (SLAM)**
- **Algorithmic Processing:** The Pi 5 ingests the bundled raw sensor data sent by the Pico 2 W over the UART connection.
- **Mapping and Pathfinding:** It runs complex navigation stacks like SLAM (Simultaneous Localization and Mapping), combining the 4x distance sensors, IMU orientation data, and camera stream to build a virtual map of its surroundings and calculate safe paths. [[1]](https://www.hackster.io/HiwonderRobot/pi-5-powerhouse-building-the-ultimate-ros-2-composite-bot-fd2bc1)

**3. Central System Orchestration**
- **Robot Operating System (ROS):** The Pi 5 runs a full Linux OS (Raspberry Pi OS) to host framework tools like ROS 2, which manage the overall timing, decision-making, and state machine of the robot. [[1]](https://www.embeddedrelated.com/showarticle/1332.php) [[2]](https://hackmd.io/@ampheo/how-does-raspberry-pi-work) [[3]](https://www.hiwonder.com/blogs/news/raspberry-pi)
- **The Decision Loop:** It makes macroscopic decisions (e.g. "there is a red block ahead, stop and turn left") and broadcasts simple command packets down to the Pico 2 W (e.g. "set servo to 45 degrees, set motor speed to 50%").

**4. Wireless Telemetry and UI**
- **User Interface:** The Pi 5 can host a web dashboard, stream live camera feeds to a phone or laptop over local Wi-Fi, or log sensor metrics directly to its micro-SD card for performance analysis. [[1]](https://www.elecrow.com/blog/home-automation-projects-with-raspberry-pi-5.html)



### What the Pico 2 W is responsible for

The Raspberry Pi Pico 2 W acts as the low-level "Brainstem" or real-time hardware controller. While the Pi 5 handles heavy computing and vision, it is poorly suited to precise microsecond-level timing because its Linux OS is constantly juggling background tasks. The Pico 2 W runs a dedicated microcontroller loop instead, making it the right tool for direct hardware execution.

**1. High-Speed Sensor Harvesting**
- **The Multiplexer Manager:** The Pico actively manages the TCA9548A multiplexer — opening Channel 0, reading a distance sensor, switching to Channel 1, reading the color sensor, and looping through all channels cleanly without dropping packets.
- **Data Serialization:** It collects raw bytes from the 4x ToF sensors, the BNO085 IMU, and the TCS34725 color module, packages them into a single text string, and streams it out via UART to the Pi 5.

**2. Precise Pulse Width Modulation (PWM) Actuation**
- **Smooth Steering Control:** It generates the exact 50Hz PWM signal needed to keep the MG90S servo locked onto its target angle without jitter.
- **Motor Speed Control:** It fires high-frequency PWM pulses to the DRV8833 motor driver to smoothly throttle the speed and direction of the GA12-N20 DC motor.

**3. Microsecond Odometry Tracking**
- **Encoder Interrupts:** As the GA12-N20 motor spins, its internal encoder generates thousands of electrical pulses per minute. The Pico 2 W uses hardware interrupts to count every pulse on GP12 and GP13 without dropping any, tracking exactly how far the wheel has rotated.

**4. Safety Interlocks (Emergency Stop)**
- Because the Pico is wired directly to the distance sensors and the motor driver, it can run low-level override code. If a VL53L0X sensor suddenly detects an obstacle at close range, the Pico can instantly cut power to the DRV8833 rather than waiting for the Pi 5 to process a camera frame and send a command.

---


## Design Rationale

Why these specific parts, rather than the alternatives.

### Compute & Control

Splitting compute across two boards was a deliberate choice rather than the default. A single Raspberry Pi 5 could technically run everything, including the servo/motor PWM, but Linux is not a real-time OS — a background process (a filesystem sync, a Wi-Fi hiccup, a garbage-collection pause) can stall a PWM update by tens of milliseconds, which is enough to make steering twitchy or let a wheel spin unchecked for a moment. Offloading motor control, encoder counting, and the safety cutoff to the Pico 2 W means those functions keep running on a dedicated loop no matter how busy the Pi 5 gets processing camera frames. The tradeoff is added wiring and a UART link to keep the two boards in sync, which we accepted in exchange for control loops that don't get starved by the vision stack.

### Drivetrain

The GA12-N20 6V 300RPM gearmotor was chosen for its combination of small size, built-in gearbox, and an onboard encoder — the encoder is the deciding factor, since it lets the Pico measure actual wheel rotation instead of assuming a PWM duty cycle maps cleanly to speed. Theoretical top speed follows v = πDN/60 (D = wheel diameter, N = motor RPM), but that number ignores load, friction, and battery sag, so once the wheel diameter is finalized we plan to measure real acceleration, top speed, and cornering behaviour and tune the operating PWM experimentally rather than just running the motor flat out.

The DRV8833 drives it. It was picked over bulkier options like the L298N specifically for its smaller footprint and better efficiency at the low voltages this build runs at — it handles both direction and PWM speed control while isolating the motor's current draw from the logic-level pins on the Pico.

Steering uses the MG90S servo rather than a second drive motor or a rack-and-pinion setup, since a servo gives direct, repeatable angle control in a small package, and its PWM signal comes straight from the Pico — so steering response stays independent of whatever the Pi 5 is doing with the camera feed.

### Power System

The robot runs on a 7.4V, 1500mAh, 35C, 2S1P LiPo (Ovonic AIR). At nominal voltage that's E = VQ = 7.4 × 2.2 = **16.28Wh** of stored energy — which matches the rating printed on the pack itself.

One deliberate design choice here: the DRV8833's motor supply (`VM`) taps the raw battery voltage directly, in parallel with the Mini560 buck converter's input, rather than running off the regulated 5V rail. That gives the motor more voltage headroom than a 5V-limited system would, at the cost of intentionally overdriving a 6V-rated motor if it were ever run at 100% duty — which is why PWM duty cycle needs to stay software-limited rather than left uncapped. In exchange, motor current never has to pass through the regulated logic rail at all, so current spikes from the motor can't sag the voltage feeding the Pi 5, Pico, or sensors. The Mini560 converter handles the regulated 5V side that actually powers the Pi 5, the Pico, and the MG90S.

### Distance & Environmental Sensing

Four VL53L0X time-of-flight sensors sit across the front and back of the chassis rather than one, so the robot gets multiple simultaneous distance readings across its field of travel instead of a single point measurement — useful for estimating position relative to walls rather than just detecting "something is close."

All VL53L0X units share the same default I²C address, which is a problem the moment you want more than one on the same bus. Rather than reflashing each sensor's address individually, a TCA9548A 8-channel I²C multiplexer sits between the Pico and the sensor network, giving each device (4x VL53L0X, the TCS34725, and the BNO085) its own isolated channel. That uses 6 of the multiplexer's 8 channels, leaving 2 free for future sensor additions without any rewiring of the existing network.

### Orientation & Colour Sensing

The BNO085 IMU adds orientation and motion data that neither the wheel encoder nor the camera can fully provide on their own — particularly useful mid-turn, when wheel odometry alone tends to drift.

The TCS34725 exists specifically so color detection doesn't depend entirely on the camera. It gives a direct, low-overhead RGB reading that the Pico can read and act on quickly, while the camera handles the spatial/visual side of track interpretation on the Pi 5. Two independent ways of sensing color is more robust than betting everything on one.

### Vision System

The Camera Module 3 is the primary visual sensor, feeding the Pi 5's computer-vision pipeline. Because Camera Module 3 uses a 15-pin FPC connector while the Pi 5's CSI ports are the newer 22-pin small-pitch style, it needs the correct 15-to-22 pin adapter cable rather than the cable the camera ships with for older Pi boards.


## Master Connection & PCB Routing List

### 1. Power Distribution Bus
- Battery Positive (+) ➡️ Mini560 IN(+) AND DRV8833 VM (Motor Power Input).
- Battery Negative (-) ➡️ Mini560 IN(-) (Establishes the main system GND rail).
- Mini560 OUT(+) [5V Rail] ➡️ Pico 2 W Pin 39 (VSYS) AND Pi 5 Pin 2/4 (5V) AND MG90S Servo Red (VCC) AND N20 Encoder VCC.
- Mini560 OUT(-) [GND Plane] ➡️ Pico Pin 38 (GND) AND Pi 5 Pin 6 (GND) AND Servo Brown (GND) AND Encoder GND AND DRV8833 GND. Flood the entire bottom layer of the PCB as a solid ground plane to connect these seamlessly.
- Pico Pin 36 (3V3 OUT Rail) ➡️ VCC / VIN pins of the TCA9548A, BNO085, TCS34725, and all 4x VL53L0X sensors.

### 2. High-Level Logic Interconnects
- Pico Pin 1 (GP0 / TX) ➡️ Pi 5 GPIO Pin 10 (RXD0 / GPIO 15)
- Pico Pin 2 (GP1 / RX) ➡️ Pi 5 GPIO Pin 8 (TXD0 / GPIO 14)
- Camera Module 3 ➡️ Connects directly to Pi 5 CAM0 or CAM1 ports via a dedicated 15-to-22 pin flexible ribbon cable. (Does not wire to the Pico 2 W.)

### 3. Sensor I2C Network
- Pico Pin 6 (GP4 / SDA) ➡️ TCA9548A SDA (add a 4.7kΩ pull-up resistor to the 3.3V trace).
- Pico Pin 7 (GP5 / SCL) ➡️ TCA9548A SCL (add a 4.7kΩ pull-up resistor to the 3.3V trace).
- TCA9548A CH0 (SD0 / SC0) ➡️ VL53L0X Distance Sensor #1 (SDA / SCL)
- TCA9548A CH1 (SD1 / SC1) ➡️ TCS34725 Color Sensor (SDA / SCL)
- TCA9548A CH2 (SD2 / SC2) ➡️ BNO085 IMU Sensor (SDA / SCL)
- TCA9548A CH3 (SD3 / SC3) ➡️ VL53L0X Distance Sensor #2 (SDA / SCL)
- TCA9548A CH4 (SD4 / SC4) ➡️ VL53L0X Distance Sensor #3 (SDA / SCL)
- TCA9548A CH5 (SD5 / SC5) ➡️ VL53L0X Distance Sensor #4 (SDA / SCL)
- Note: leave all VL53L0X XSHUT pins completely disconnected on the PCB — they pull high naturally.

### 4. Drivetrain Control (Motors, Encoders, & Servos)
- Pico Pin 29 (GP22) ➡️ MG90S Servo Orange (Signal Pin) — bypasses the driver entirely.
- Pico Pin 11 (GP8) ➡️ DRV8833 IN1 (Motor Drive Forward PWM)
- Pico Pin 12 (GP9) ➡️ DRV8833 IN2 (Motor Drive Reverse Pin)
- DRV8833 OUT1 / OUT2 ➡️ GA12-N20 Motor Pin 1 & Pin 2 (Motor Power Leads)
- GA12-N20 Motor Pin 5 (Encoder A) ➡️ Pico Pin 16 (GP12)
- GA12-N20 Motor Pin 6 (Encoder B) ➡️ Pico Pin 17 (GP13)

