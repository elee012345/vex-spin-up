# x drive auton
copied and pasted from my comments in the code 
if i didn't write this down i would get so confused next year as to what i was doing now

    robot wheels looks like this:
  /   \
  \   /

  they vector outward at 45 degrees each
 however, we want to drive forward a certain amount of inches
not driving 45 degrees in a direction
if you extend everything out then you get a square
with a line going from one corner to another that is as far as you want
your robot to drive.
You get a 45 45 90 triangle, where the legs are each x and the hypotenuse is x root 2
that means each of the legs is the distance you want to go divided by root 2
so we have each of the robot motors turn that far instead

then we multiply by 1.1 to overshoot a little bit because our motors have play and are bad

