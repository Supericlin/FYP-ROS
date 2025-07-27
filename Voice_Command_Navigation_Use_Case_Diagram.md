# Voice Command Navigation System - Use Case Diagram

## System Overview
This Use Case Diagram represents the voice command navigation system for the ROS2-based robot, showing how users can specify destinations through voice commands from mobile applications or AI agents.

## Use Case Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           VOICE COMMAND NAVIGATION SYSTEM                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────┐                    ┌─────────────────┐                    │
│  │   MOBILE USER   │                    │   AI AGENT      │                    │
│  │                 │                    │                 │                    │
│  │ • User          │                    │ • AI Assistant  │                    │
│  │ • Caregiver     │                    │ • Voice Bot     │                    │
│  │ • Family Member │                    │ • Smart Speaker │                    │
│  └─────────────────┘                    │ • ESP32-S3      │                    │
│           │                             └─────────────────┘                    │
│           │                                       │                            │
│           │                                       │                            │
│           ▼                                       ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────────────┐ │
│  │                        VOICE COMMAND INTERFACE                             │ │
│  │                                                                             │ │
│  │  ┌─────────────────┐              ┌─────────────────┐                      │ │
│  │  │   MOBILE APP    │              │   AI AGENT      │                      │ │
│  │  │   (Android)     │              │   INTERFACE     │                      │ │
│  │  │                 │              │                 │                      │ │
│  │  │ • Voice Input   │              │ • Voice Input   │                      │ │
│  │  │ • Speech Recog. │              │ • MQTT Bridge   │                      │ │
│  │  │ • Location List │              │ • Command Trans.│                      │ │
│  │  └─────────────────┘              └─────────────────┘                      │ │
│  └─────────────────────────────────────────────────────────────────────────────┘ │
│           │                                       │                            │
│           │                                       │                            │
│           ▼                                       ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────────────┐ │
│  │                        COMMAND PROCESSING LAYER                            │ │
│  │                                                                             │ │
│  │  ┌─────────────────┐              ┌─────────────────┐                      │ │
│  │  │   MQTT BROKER   │              │   ROS2 NODES    │                      │ │
│  │  │                 │              │                 │                      │ │
│  │  │ • Message Route │              │ • Navigation    │                      │ │
│  │  │ • Topic Mgmt    │              │ • Goal Process  │                      │ │
│  │  │ • Status Pub    │              │ • Status Monitor│                      │ │
│  │  └─────────────────┘              └─────────────────┘                      │ │
│  └─────────────────────────────────────────────────────────────────────────────┘ │
│           │                                       │                            │
│           │                                       │                            │
│           ▼                                       ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────────────┐ │
│  │                           ROBOT NAVIGATION SYSTEM                          │ │
│  │                                                                             │ │
│  │  ┌─────────────────┐              ┌─────────────────┐                      │ │
│  │  │   NAV2 STACK    │              │   STATUS FEEDBACK│                      │ │
│  │  │                 │              │                 │                      │ │
│  │  │ • Path Planning │              │ • Goal Reached  │                      │ │
│  │  │ • Localization  │              │ • Navigation    │                      │ │
│  │  │ • Navigation    │              │ • Error Status  │                      │ │
│  │  └─────────────────┘              └─────────────────┘                      │ │
│  └─────────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Detailed Use Cases

### 1. **Mobile App Voice Command Use Case**

#### Primary Actor: Mobile User
#### Supporting Actors: MQTT Broker, ROS2 Navigation System

**Main Success Scenario:**
1. **User opens mobile app** and taps voice input button
2. **App starts speech recognition** and prompts user to speak destination
3. **User speaks destination** (e.g., "Go to McDonald's", "Navigate to toilet")
4. **App processes voice input** and matches to predefined locations
5. **App sends MQTT command** to robot system via `robot/control` topic
6. **Robot receives command** and starts navigation
7. **Robot provides status updates** via `robot/status` topic
8. **App displays navigation progress** to user

**Alternative Flows:**
- **A1:** Voice recognition fails → App shows manual location selection dialog
- **A2:** Unrecognized location → App shows error and suggests alternatives
- **A3:** Connection lost → App attempts reconnection and shows status

**Preconditions:**
- Mobile app is connected to MQTT broker
- Robot system is operational
- Voice recognition service is available

**Postconditions:**
- Navigation goal is sent to robot
- User receives confirmation of destination

### 2. **AI Agent Voice Command Use Case**

#### Primary Actor: AI Agent (ESP32-S3)
#### Supporting Actors: MQTT Broker, ROS2 Navigation System

**Main Success Scenario:**
1. **AI Agent receives voice command** from user (e.g., "去書房", "去睡房")
2. **Agent processes Chinese voice input** and translates to English commands
3. **Agent publishes MQTT message** to `robot/control` topic
4. **Robot receives navigation command** and starts navigation
5. **Robot sends status updates** via `robot/status` topic
6. **Agent monitors navigation progress** and provides feedback
7. **Agent receives goal reached notification** and plays success sound
8. **Agent triggers wake word** for next interaction

**Alternative Flows:**
- **A1:** Command not recognized → Agent asks for clarification
- **A2:** Navigation fails → Agent reports error and suggests alternatives
- **A3:** Connection issues → Agent attempts reconnection

**Preconditions:**
- AI Agent is connected to MQTT broker
- Robot system is operational
- Voice processing is active

**Postconditions:**
- Navigation command is processed
- User receives audio/visual feedback

### 3. **Navigation Control Use Cases**

#### 3.1 **Pause Navigation**
- **Trigger:** User says "pause navigation" or "navPause"
- **Action:** Robot pauses current navigation
- **Status:** Navigation can be resumed

#### 3.2 **Stop Navigation**
- **Trigger:** User says "stop navigation" or "navStop"
- **Action:** Robot cancels current navigation
- **Status:** Robot returns to idle state

#### 3.3 **Resume Navigation**
- **Trigger:** User says "continue navigation" or "navCon"
- **Action:** Robot resumes paused navigation
- **Status:** Navigation continues to destination

### 4. **Status Monitoring Use Cases**

#### 4.1 **Goal Reached Notification**
- **Trigger:** Robot reaches destination
- **Action:** System publishes "goalReached" status
- **Response:** Mobile app and AI Agent provide success feedback

#### 4.2 **Navigation Progress Updates**
- **Trigger:** Navigation status changes
- **Action:** System publishes progress updates
- **Response:** User interfaces update progress indicators

## Supported Voice Commands

### Mobile App Commands:
- "Go to McDonald's"
- "Navigate to toilet"
- "Go to entrance"
- "Go to Watson's"
- "Go to information counter"
- "Go to Starbucks"

### AI Agent Commands (Chinese):
- "去書房" (Go to study room)
- "去睡房" (Go to bedroom)
- "去客廳" (Go to living room)
- "去飯廳" (Go to dining room)
- "去洗手間" (Go to bathroom)
- "導航暫停" (Pause navigation)
- "導航停止" (Stop navigation)

## System Integration Points

### MQTT Topics:
- **`robot/control`**: Navigation commands from mobile app and AI agent
- **`robot/status`**: Robot status updates and notifications
- **`robot/command`**: Wake-up commands and system control

### ROS2 Nodes:
- **`mqtt_navigation_node`**: Processes MQTT commands and sends to Nav2
- **`android_navigation_node`**: Handles mobile app navigation requests
- **Nav2 Stack**: Core navigation functionality

### Communication Flow:
```
User Voice → Mobile App/AI Agent → MQTT Broker → ROS2 Node → Nav2 → Robot Movement
```

## Error Handling

### Connection Issues:
- Automatic reconnection attempts
- Fallback to manual selection
- Status indicators for connection state

### Voice Recognition Failures:
- Manual location selection dialog
- Error messages with suggestions
- Retry mechanisms

### Navigation Failures:
- Error reporting to user
- Alternative route suggestions
- System status notifications

This Use Case Diagram provides a comprehensive view of how voice commands are integrated into your robot navigation system, showing the interaction between users, mobile applications, AI agents, and the underlying ROS2 navigation infrastructure. 