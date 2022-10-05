# x drive auton
copied and pasted from my comments in the code 
if i didn't write this down i would get so confused next year as to what i was doing now

nice fancy picture i drew explaning it: https://github.com/elee012345/vex-spin-up/blob/main/auton%20testing/explanation.png

robot wheels looks like this:
```  
  /   \
  \   /
```

They vector outward at 45 degrees each.
However, we want to drive forward a certain amount of inches, not driving 45 degrees in a direction.
If you extend everything out then you get a square with a line going from one corner to another that is as far as you want your robot to drive.
You get a 45 45 90 triangle, where the legs are each x and the hypotenuse is x root 2.
That means each of the legs is the distance you want to go divided by root 2, so we have each of the robot motors turn that far instead.
Then we multiply by 1.1 to overshoot a little bit because our motors have play and are bad and also because the floor has friction.

[![fancy little explanation picture from ms paint](https://raw.githubusercontent.com/elee012345/vex-spin-up/main/auton%20testing/explanation.png)](https://github.com/elee012345/vex-spin-up/blob/main/auton%20testing/explanation.png)