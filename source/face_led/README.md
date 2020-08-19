# face_led

ROS led controller for the head of ATRV robot.

# Launch

Launch interactable node with:

```
rosrun face_led led_controller.py
```

And launch a demo of all possible emotions using:

```
python demo.py
```

# Usage

Once led_controller is runnning it becomes subscribed to:
```
/face_emotion
```
and
```
/face_talk_anim
```
which are a String and Float32 respectivelly and allow setting of emotional presets and talking animations.

The string can be one of these: blank, neutral, neutral_left, neutral_right,furious, mad, stupid, sad, miserable, happy, nauseated, surprised, loading. 

The float corresponds to the time of seconds to animate mouth after receiving message.

Wheel of emotions: https://i.redd.it/zs6xww1s8f911.jpg
