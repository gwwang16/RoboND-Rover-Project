import numpy as np
import time
# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function


def decision_step_backup(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # run along with the left side
                Rover.steer = np.clip(
                    np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)

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
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover


def obstacle_avoidance(Rover):
    '''
    obstacle avoidance.
    If there are no obstacles in front of rover, move forward;
    if obstacles are detected, turn away from it until it is no
    longer in front of rover.
    '''
    obstacle = Rover.vision_image[:, :, 0]
    img_shape = obstacle.shape
    x_origin = np.int_(img_shape[0] - Rover.bottom_offset)
    y_origin = np.int_(img_shape[1] / 2)
    if np.maximum(obstacle[x_origin - 5:x_origin + 5, y_origin:y_origin + 20]) > 200:
        # print(obstacle[x_origin-5:x_origin+5, y_origin:y_origin+20].nonzero())
        print('There is obstacle in front of rover')
    return None


def return_starting_point(Rover):
    '''Return to the starting pint when the rover complete some special tasks'''
    start_point = np.array((100, 100))
    # Calculate angle and distance from current point to start point.
    rover_point = np.array(Rover.pos)
    dist = np.sqrt(np.sum((rover_point - start_point)**2))
    rover_point = np.array(Rover.pos)
    angle = np.arctan2(start_point[1] - rover_point[1],
                       start_point[0] - rover_point[0])
    angle_error = angle - Rover.yaw
    turn_state = 0
    if turn_state == 0:
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        # Turn the rover toward to the start point
        elif (Rover.vel <= 0.2) and (angle_error > 1):
            # Now we're stopped and we have vision data to see if there's a path forward
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees,
            # when stopped it will induce 4-wheel turning
            Rover.steer = np.clip(angle_error, -15, 15)
        else:
            turn_state = 1

    if dist > 5:
        # Then execute movement toward start point
        # with obstacle avoidance
        # nav_angle_q25 = np.percentile(
         #   np.array(Rover.nav_angles * 180 / np.pi), [75, 25])
        nav_angle_q75 = np.percentile(
            np.array(Rover.nav_angles * 180 / np.pi), 75)
        nav_angle_q25 = np.percentile(
            np.array(Rover.nav_angles * 180 / np.pi), 25)
        print(nav_angle_q75)
        nav_angle_low = np.maximum(nav_angle_q25, -15)
        nav_angle_upper = np.minimum(nav_angle_q75, 15)
        Rover.steer = np.clip(angle_error, nav_angle_low, nav_angle_upper)
        Rover.brake = 0
        Rover.throttle = Rover.throttle_set
    else:
        Rover.brake = Rover.brake_set
        Rover.throttle = 0
        Rover.steer = 0
    return Rover


def decision_step(Rover):
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
                            print("time_start: ", Rover.time_start, 'time_now: ', time_now)
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
