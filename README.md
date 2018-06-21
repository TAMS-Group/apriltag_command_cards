# Control your robot using AprilTag command cards!
- Print a card for each command
- Show command cards to the robot to trigger commands, control continuous actions, or to give directions to the robot
- Temporal filtering is applied for commands to prevent unintentional re-triggering (e.g. if the card disappears for just one or two frames, or if a card can't be detected for a short time due to motion blur)

#### Topics
- A global command topic is provided on which a string message is published whenever a command is triggered
- In addition, more detailed per-card topics are provided as well (`<card>/command`, `<card>/visibility`, and `<card>/direction`)
- If a command is triggered, a message is published on the global command topic and another one is published on the card's `<card>/command` topic
- The `<card>/visibility` indicates whether the corresponding card is currently visible or not (for controlling continuous actions)
- The `<card>/direction` gives the direction vector from the camera to the command card if the card is currently visible, and a null vector otherwise (e.g. for making the robot look or drive towards the command card)

#### Configuration
- Add a list of AprilTag IDs and command names to `/config/apriltag_command_cards.yml`
- Using `/config/apriltag_command_cards.yml`, you can also adjust the timeout of the temporal filter
- If you're starting the node directly without the launch files, or if you're writing your own launch file, you can also put your configuration file somewhere else (see included launch file for more information)

#### Launching
- To start the command card node, you can use the included launch file: `roslaunch apriltag_command_cards 	apriltag_command_cards.launch`