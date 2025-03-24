# X-Drive Autonomous Navigation

This document explains the mathematical principles behind our X-drive autonomous movement calculations.

## Wheel Configuration

Our X-drive uses four wheels positioned at 45-degree angles:

```
  /   \
  \   /
```

## Mathematical Principles

To achieve precise autonomous movement, we need to convert desired straight-line distances into appropriate wheel rotations, accounting for the 45-degree wheel vectors.

### Key Concepts:

1. The wheels are vectored outward at 45° angles
2. To drive straight, we need to calculate compensated distances due to this 45° offset
3. Uses 45-45-90 triangle principles for distance calculations
4. Applies a 1.1x multiplier to account for mechanical play and friction

### Calculation Method

1. For any desired straight-line distance (d):
   - Each wheel needs to turn d/√2 distance
   - This comes from the 45-45-90 triangle relationship where:
     - Hypotenuse = d (desired distance)
     - Legs = d/√2 (actual wheel travel distance)

2. Final wheel rotation = (d/√2) * 1.1
   - 1.1 multiplier compensates for mechanical inefficiencies

## Visual Explanation

[![X-Drive Movement Calculation Diagram](https://raw.githubusercontent.com/elee012345/vex-spin-up/main/auton%20testing/explanation.png)](https://github.com/elee012345/vex-spin-up/blob/main/auton%20testing/explanation.png)

The diagram shows how the 45° wheel vectors create a geometric relationship we can use to calculate precise movement distances.


<details>
  <summary>old description</summary>
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
</details>