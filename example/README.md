<a id="ur5e_moveit_client"></a>

# ur5e\_moveit\_client

<a id="ur5e_moveit_client.UR5eMoveItClient"></a>

## UR5eMoveItClient

```python
class UR5eMoveItClient(Node)
```

UR5eMoveItClient is a ROS 2 node that interfaces with MoveIt 2 to control a UR5e robotic arm.
It abscracts the MoveIt 2 action interface and provides methods to move the robot to specified
joint positions and poses. It also provides methods to retrieve the current joint positions 
and the position of a specified link in the robot.

<a id="ur5e_moveit_client.UR5eMoveItClient.get_joint_names"></a>

#### get\_joint\_names

```python
def get_joint_names()
```

Retrieves the joint names from the MoveIt interface.

**Returns**:

- `List[str]` - The list of joint names.

<a id="ur5e_moveit_client.UR5eMoveItClient.move_to_joints"></a>

#### move\_to\_joints

```python
def move_to_joints(joint_positions: list) -> GoalStatus
```

Moves the UR5e to the specified joint positions.

**Arguments**:

- `joint_positions` _list_ - A list of 6 floats representing the joint positions.
  

**Returns**:

- `GoalStatus` - The status of the goal.

<a id="ur5e_moveit_client.UR5eMoveItClient.move_to_pose"></a>

#### move\_to\_pose

```python
def move_to_pose(pose: list) -> GoalStatus
```

Moves the robot to the specified pose.

This function sends a goal message to the action server to move the robot
to the desired pose. It waits for the action server to accept the goal and
then waits for the result of the action.

**Arguments**:

- `pose` _list_ - The target pose for the robot. A pose is a list of 7 floats,
  where the first three floats represent the x, y, and z coordinates of the
  position, and the last four floats represent the orientation in quaternion
  

**Returns**:

- `GoalStatus` - The status of the goal after execution. It returns
  `GoalStatus.STATUS_SUCCEEDED` if the goal was achieved successfully,
  otherwise it returns `GoalStatus.STATUS_ABORTED`.

<a id="ur5e_moveit_client.UR5eMoveItClient.move_to_pose_async"></a>

#### move\_to\_pose\_async

```python
def move_to_pose_async(pose: list) -> GoalStatus
```

Asynchronously moves the robot to the specified pose.
This method prepares a goal message with the given pose and sends it to the action server.
If there is a current goal being executed, it attempts to cancel it before sending the new goal.
A small delay is added to ensure the system processes the cancellation request.

**Arguments**:

- `pose` _list_ - The target pose for the robot.

**Returns**:

- `GoalStatus` - The status of the goal after being sent to the action server. It returns
  STATUS_SUCCEEDED if the goal is accepted and STATUS_ABORTED if the goal is rejected.

<a id="ur5e_moveit_client.UR5eMoveItClient.get_joint_positions"></a>

#### get\_joint\_positions

```python
def get_joint_positions() -> list
```

Retrieves the current joint positions of the robot.
This method creates a client to call the 'get_planning_scene' service and
requests the robot's current state. It waits for the service to be available
and then sends the request. If the service call is successful, it returns
the joint positions of the robot. If the service call fails, it logs an
error message and returns an empty list.

**Returns**:

- `list` - A list of joint positions if the service call is successful,
  otherwise an empty list.

<a id="ur5e_moveit_client.UR5eMoveItClient.get_link_position"></a>

#### get\_link\_position

```python
def get_link_position(link_name: str)
```

Retrieves the position of the specified link.

**Arguments**:

- `link_name` _str_ - The name of the link.
  

**Returns**:

- `tuple` - A tuple containing the position (x, y, z) and orientation (quaternion) of the link.

<a id="example"></a>

# example

As an examples, see the [example.py](example.py) file.
