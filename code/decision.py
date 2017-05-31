import numpy as np
import time
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function

def calculate_angle_error(x_1, x_2, yaw):
    '''Translate arctan angle in range of [0, 2pi],
    calculate angle error between current yaw and target angle.
    x1 is the target point, x2 is the current point,
    yaw is rover yaw in degree.'''
    start_point = np.array(x_1)
    rover_point = np.array(x_2)

    angle_rad = np.arctan2(start_point[1] - rover_point[1], start_point[0] - rover_point[0])
    # transfer the negative angle_rad to [pi, 2pi]
    if angle_rad < 0:
        angle_rad = angle_rad + 2* np.pi
    # transfer radians to degree
    angle_degree = angle_rad * 180 / np.pi

    angle_error = angle_degree - yaw

    return angle_error


def return_starting_point(Rover):
    '''Return to the starting pint when the rover complete some special tasks'''
    # Calculate angle and distance from current point to start point.
    Rover.angle_error = calculate_angle_error(Rover.start_point, Rover.pos, Rover.yaw)

    if np.absolute(Rover.angle_error) > 0.5 and not Rover.turn_to_start:
        print("Turning tarward o home point!")
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # Turn the rover toward to the start point
        elif Rover.vel <= 0.2:
            if np.absolute(Rover.angle_error) > 1:
                # Now we're stopped and we have vision data to see if there's a path forward
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees,
                # when stopped it will induce 4-wheel turning
                Rover.steer = np.clip(Rover.angle_error, -15, 15)
                # Update avaliable condition
                Rover.angle_error = calculate_angle_error(Rover.start_point, Rover.pos, Rover.yaw)

            else:
                Rover.steer = 0
                Rover.turn_to_start = True

    # If angle error is smaller, execute back home movement
    else:
        if len(Rover.nav_angles) >= Rover.stop_forward:
            # Move toward start point with obstacle avoidance
            # Using 25% and 75% larger nav_angle to be steer boundary,
            # Rover steer angle is angle error between yaw and angle to home point
            nav_angle_q75 = np.percentile(
                np.array(Rover.nav_angles * 180 / np.pi), 75)
            nav_angle_q25 = np.percentile(
                np.array(Rover.nav_angles * 180 / np.pi), 25)
            nav_angle_low = np.maximum(nav_angle_q25, -15)
            nav_angle_upper = np.minimum(nav_angle_q75, 15)

            Rover.steer = np.clip(Rover.angle_error, nav_angle_low, nav_angle_upper)
            Rover.angle_error = calculate_angle_error(Rover.start_point, Rover.pos, Rover.yaw)

            Rover.brake = 0
            # Use a larger throttle to back home
            if Rover.vel <= (Rover.max_vel * 2):
                Rover.throttle = Rover.throttle_set * 2
            else:
                Rover.throttle = 0
            print("Angle error: ", Rover.angle_error)
            print("Complete task, backing home!")

        elif len(Rover.nav_angles) < Rover.stop_forward:
            # Set mode to "stop" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'stop'
            Rover = stop_mode(Rover)

    return Rover


def pickup_mode(Rover):
    '''This function controls rover to pick up samples'''
    if len(Rover.rock_angles) > 0:
        if Rover.near_sample and not Rover.picking_up:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.mode = 'stop'
            Rover.send_pickup = True
        else:
            Rover.steer = np.clip(
                np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)

            # low speed mode
            if Rover.vel > (Rover.max_vel / 2):
                Rover.throttle = 0
                Rover.brake = 0.2

            else:
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
    else:
        Rover.mode = 'stop'
    return Rover


def stop_mode(Rover):
    '''Stop and turn the wheel when there is no path forward'''
    if len(Rover.rock_angles) > 0:
        Rover.steer = np.clip(
            np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
        Rover.mode = 'pickup'

    elif Rover.mode == 'stop':
        # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Now we're stopped and we have vision data to see if there's a path forward
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees,
                # when stopped the next line will induce 4-wheel turning
                Rover.steer = -15  # Could be more clever here about which way to turn
            # If we're stopped but see sufficient navigable terrain in front then go!
            if len(Rover.nav_angles) >= Rover.go_forward:
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set
                # Release the brake
                Rover.brake = 0
                # Set steer to mean angle
                Rover.steer = np.clip(
                    np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                Rover.mode = 'forward'
    return Rover


def decision_step_v1(Rover):
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.samples_found < 3 or Rover.home_dist > 5:
            # Check for Rover.mode status
            if Rover.mode == 'pickup':
                if len(Rover.rock_angles) > 0:
                    if Rover.near_sample and not Rover.picking_up:
                        Rover.brake = Rover.brake_set / 5
                        Rover.throttle = 0
                        Rover.mode = 'stop'
                        Rover.send_pickup = True
                        # Rover.rock_pick += 1
                        print(Rover.samples_found)
                    else:
                        Rover.steer = np.clip(
                            np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
                        # low speed mode
                        if Rover.vel > (Rover.max_vel / 2):
                            Rover.throttle = 0
                            Rover.brake = 1
                        else:
                            Rover.throttle = Rover.throttle_set
                            Rover.brake = 0
                else:
                    Rover.mode = 'stop'

            elif Rover.mode == 'forward':
                # Check if there is rock in front of rover
                if len(Rover.rock_angles) > 0:
                    Rover.steer = np.clip(
                        np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'pickup'

                # Check the extent of navigable terrain
                elif len(Rover.nav_angles) >= Rover.stop_forward:
                    # If mode is forward, navigable terrain looks good
                    # and velocity is below max, then throttle

                    if Rover.vel < Rover.max_vel:
                        # Set a larger throttle value at begginging
                        if Rover.vel < 0.2 and Rover.mode == 'forward':
                            Rover.throttle = Rover.throttle_set * 2
                            time_now = time.time() - Rover.time_start
                            print("time_start: ", Rover.time_start,
                                  'time_now: ', time_now)
                            print('Mode: ', Rover.mode)
                            # If rover stuck in low speed in forward mode.
                            # try to use max throttle first
                            if 5 <= time_now < 8:
                                Rover.throttle = Rover.max_throttle
                            # try to turn the rover
                            elif 8 <= time_now <= 10:
                                Rover.throttle = 0
                                Rover.brake = 0
                                Rover.steer = -15
                            if time_now > 10:
                                Rover.time_start = time.time()
                                time_now = 0
                        # Set throttle value to throttle setting
                        else:
                            Rover.throttle = Rover.throttle_set
                    else:  # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    # run along with the left side, using the 75% large nav angle.
                    nav_angle_q75 = np.percentile(
                        np.array(Rover.nav_angles * 180 / np.pi), 75)
                    Rover.steer = np.clip(nav_angle_q75, -15, 15)
                   # Rover.steer = np.clip(
                    #    np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)

                    # Calculate distance from rover position to home
                    start_point = np.array(Rover.start_point)
                    # Calculate angle and distance from current point to start point.
                    rover_point = np.array(Rover.pos)
                    Rover.home_dist = np.sqrt(
                        np.sum((rover_point - start_point)**2))

                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

            # If we're already in "stop" mode then make different decisions
            elif Rover.mode == 'stop':
                # If we're in stop mode but still moving keep braking
                if len(Rover.rock_angles) > 0:
                    Rover.steer = np.clip(
                        np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'pickup'

                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward:
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees,
                        # when stopped the next line will induce 4-wheel turning
                        Rover.steer = -15  # Could be more clever here about which way to turn
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    if len(Rover.nav_angles) >= Rover.go_forward:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(
                            np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                        Rover.mode = 'forward'
        # Just to make the rover do something
        # even if no modifications have been made to the code
        else:
            # Rover = return_starting_point(Rover)
            # Simple strategy to back home
            Rover.throttle = 0
            Rover.steer = 0
            Rover.brake = Rover.brake_set
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    return Rover


def decision_step(Rover):
    '''decision process'''
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.samples_found < 1:
            # Check for Rover.mode status
            if Rover.mode == 'pickup':
                Rover = pickup_mode(Rover)

            elif Rover.mode == 'forward':
                # Check if there is rock in front of rover
                if len(Rover.rock_angles) > 0:
                    Rover.mode = 'pickup'

                # Check the extent of navigable terrain
                elif len(Rover.nav_angles) >= Rover.stop_forward:
                    # If mode is forward, navigable terrain looks good
                    # and velocity is below max, then throttle
                    if Rover.vel < Rover.max_vel:
                        # Set a larger throttle value at begginging
                        if Rover.vel < 0.2 and Rover.mode == 'forward' and not Rover.picking_up:
                            Rover.throttle = Rover.throttle_set * 2
                            # Set a stuck time counting
                            time_counting = time.time() - Rover.time_start
                            print('Stucking time counting: ', time_counting)
                            print('Rover mode: ', Rover.mode)
                            # If rover stuck in low speed in forward mode.
                            # try to use max throttle first
                            if 5 <= time_counting < 8:
                                Rover.throttle = Rover.max_throttle
                                print(
                                    "Stuck here, trying max throttle to break through!")
                            # try to turn the rover
                            elif 8 <= time_counting <= 10:
                                Rover.throttle = 0
                                Rover.brake = 0
                                Rover.steer = -15
                                print("Stuck here, trying turning around!")
                            if time_counting > 10:
                                # Reset stuck time counting
                                Rover.time_start = time.time()
                                time_counting = 0
                        # Set throttle value to throttle setting
                        else:
                            Rover.throttle = Rover.throttle_set
                    else:  # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0

                    # Set steering to average angle clipped to the range +/- 15
                    # run along with the left side, using the 75% large nav angle.
                    nav_angle_q75 = np.percentile(
                        np.array(Rover.nav_angles * 180 / np.pi), 75)
                    Rover.steer = np.clip(nav_angle_q75, -15, 15)

                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

            # If we're already in "stop" mode then make different decisions
            elif Rover.mode == 'stop':
                if len(Rover.rock_angles) > 0:
                    Rover.steer = np.clip(
                        np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'pickup'
                # If we're in stop mode but still moving keep braking
                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2:
                    # Now we're stopped and we have vision data to see if there's a path forward
                    if len(Rover.nav_angles) < Rover.go_forward:
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees,
                        # when stopped the next line will induce 4-wheel turning
                        Rover.steer = -15  # Could be more clever here about which way to turn
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    if len(Rover.nav_angles) >= Rover.go_forward:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(
                            np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                        Rover.mode = 'forward'

        # If all samples are found
        else:
            # Calculate angle and distance from current point to start point.
            start_point = np.array(Rover.start_point)
            rover_point = np.array(Rover.pos)
            Rover.home_dist = np.sqrt(np.sum((rover_point - start_point)**2))
            if Rover.home_dist > 3:
                Rover = return_starting_point(Rover)
            # Simple strategy to back home
            else:
                Rover.throttle = 0
                Rover.steer = 0
                if Rover.vel > 0.5:
                    Rover.brake = 1
                else:
                    Rover.brake = Rover.brake_set

                print("Fineshed the task!")

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    return Rover

