# MUCU, A Smart Automobile. Micro-controller Project.

**Youtube demo:**

[![Watch the video](https://img.youtube.com/vi/IHWIqOERDno/0.jpg)](https://www.youtube.com/watch?v=IHWIqOERDno)

## Content:
- [Presentation](/Presentation/presentation.pdf)

## Features:
1. **Bluetooth Control**:
   - Precise movement control. 
   - Commands sent via bluetooth.
   - Commands include: forward, back, left, right, and stop.
2. **Voice Control**:
   - Uses advanced Google speech-to-text technology to interpret spoken commands.
   - Commands are seamlessly transmitted via Bluetooth.
3. **Obstacle Avoidance**:
   - Uses a servo motor and an ultrasonic sensor.
   - Obstacle detection at 20 cm.
   - Iterative process ensures continuous obstacle avoidance.
   - **Dynamic direction determination to reach destination.**
   - Can be used for autonomous navigation & maze solving.
4. **Camera Control**:
   - Integrates a high-resolution camera with a WiFi module for real-time streaming.
   - Enhanced with an LED flashlight.
   - Remote surveillance and exploration.
5. **GPS Control**:
   - Precise navigation to destination latitude and longitude coordinate.
   - **Bot is released facing north initially.**
   - Constantly compares current location to determine route.
   - **Monitored via Bluetooth.**
   - Provides status updates including satellite locking and navigation progress.

**Example Cases for GPS Control (Current Latitude: 23.726 N, Longitude: 90.3905 E):**
1. Destination : (23.727 N, 90.3905 E)
- Bot moves north towards the destination.(No turn taken)
2. Destination : (23.724 N, 90.3905 E)
- Bot travels south toward the destination.(No turn taken)
3. Destination : (23.724 N, 90.38 E)
- Bot initially moves south, then turns left (west) when the latitude matches, and proceeds west to destination.
4. Destination : (23.726 N, 90.40 E)
- Bot turns right when the latitude aligns, then advances east to the destination.
5. Destination : (23.726 N, 90.3905 E)
- Bot recognizes arrival at the destination and remains stationary.