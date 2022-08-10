# from PIL import Image
from ai2thor.controller import Controller
from Levenshtein import distance as lev
import networkx as nx
import numpy as np


def edit_input(user_input):
    temp = user_input.split(' and ')
    cmd = []
    for i in temp:
        lowi = i.lower()
        cmd.append(lowi)
    return cmd


def pickup(object_id):
    return controller.step(
        action="PickupObject",
        objectId=object_id,
        forceAction=True,
        manualInteract=False
    )


def putdown(object_id):
    return controller.step(
        action="PutObject",
        objectId=object_id,
        forceAction=True,
        placeStationary=True
    )


def show_objectid(num):
    n = 0
    while n < num:
        if event.metadata["objects"][n]["visible"] is True:
            print(event.metadata["objects"][n]["objectId"])
            # print(event.metadata["objects"][n]["visible"])
        n += 1


# def save_img(num):
#     file_name = "image_" + str(num) + ".jpeg"
#     frame = controller.last_event.frame  # the frame image from the last event
#     pil_img = Image.fromarray(frame)  # create an Image object of the frame data
#     pil_img.save(f"/Users/evelynvu/Desktop/AI2THOR_SAMPLE_IMAGES/{file_name}",
#                  format="jpeg")  # write the Image object into a jpeg at location FILENAME


def generate_navigation_graph(self):
    """
    Generate navigation graph: We construct a directed graph with nodes representing agent
    position and rotation. For every occupiable grid point on the map, we create four nodes for each orientation.
    Orientation nodes at a single occupiable point are connected with directed edges for turns.
    Occupiable positions are connected with directed edges that preserve orientation.
    """

    event = self.controller.step(action="GetReachablePositions")
    p = event.metadata["actionReturn"]

    ng = nx.DiGraph()
    rotations = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    for idx in range(len(p)):
        for rx, rz in rotations:
            ng.add_node((idx, rx, rz))
    for idx in range(len(p)):
        for irx, irz in rotations:
            for jrx, jrz in rotations:
                if irx + jrx == 0 or irz + jrz == 0:
                    continue  # antipodal or identical
                ng.add_edge((idx, irx, irz), (idx, jrx, jrz))
        for jdx in range(len(p)):
            if idx == jdx:
                continue
            rx = rz = None
            if np.isclose(p[idx]["z"] - p[jdx]["z"], 0):
                if np.isclose(p[idx]["x"] - p[jdx]["x"], self.grid_size):
                    rx = -1
                    rz = 0
                elif np.isclose(p[idx]["x"] - p[jdx]["x"], -self.grid_size):
                    rx = 1
                    rz = 0
            elif np.isclose(p[idx]["x"] - p[jdx]["x"], 0):
                if np.isclose(p[idx]["z"] - p[jdx]["z"], self.grid_size):
                    rx = 0
                    rz = -1
                elif np.isclose(p[idx]["z"] - p[jdx]["z"], -self.grid_size):
                    rx = 0
                    rz = 1
            if rx is not None and rz is not None:
                ng.add_edge((idx, rx, rz), (jdx, rx, rz))
    self.navigation_graph = ng
    self.navigation_points = p


controller = Controller(
    agentMode="default",
    visibilityDistance=1.5,
    scene="FloorPlan212",

    # step sizes
    gridSize=0.25,
    snapToGrid=True,
    rotateStepDegrees=90,

    # image modalities
    renderDepthImage=False,
    renderInstanceSegmentation=False,

    # camera properties
    width=800,
    height=800,
    fieldOfView=90
)

# creating a list of all object in the room
obj_list = []
for obj in controller.last_event.metadata["objects"]:
    obj_list.append(obj["objectType"].lower())
    # print(obj["objectType"])
numOfObj = len(obj_list)

obj_dict = {}
for obj in controller.last_event.metadata["objects"]:
    obj_dict.update({obj["objectType"].lower():obj["position"]})

exit_program = False
count = 0

while exit_program is False:
    # ask for input
    inp = input("Next step: ")
    # analyze the input
    cmd = edit_input(inp)

    # no input / misspelled input
    if len(cmd) == 0:
        print("I do not quite understand... Please try again!")

    # correct input
    while len(cmd) != 0:
        count += 1
        # stop the simulator
        if cmd[0] == "quit":
            exit_program = True
            break

        # move commands
        if cmd[0].find('move') != -1:
            # move forward
            if cmd[0].find('forward') != -1:
                event = controller.step("MoveAhead")
                controller.step("Pass")
                cmd.pop(0)
                # save_img(count)
                positions = controller.step(action="GetReachablePositions").metadata["actionReturn"]
                print(positions)
                print(event.metadata["objects"][2]["position"])
                # for pos in positions:
                #     for obj in list(obj_dict.values()):
                #         if obj['x'] == pos['x'] and obj['y'] == pos['y'] and obj['z'] == pos['z']:
                #             print("true")
                # show_objectid(numOfObj)
                continue

            # move backward
            elif cmd[0].find('back') != -1:
                event = controller.step("MoveBack")
                controller.step("Pass")
                cmd.pop(0)
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # move to the left
            elif cmd[0].find('left') != -1:
                event = controller.step("MoveLeft")
                controller.step("Pass")
                cmd.pop(0)
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # move to the right
            elif cmd[0].find('right') != -1:
                event = controller.step("MoveRight")
                controller.step("Pass")
                cmd.pop(0)
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            else:
                print("I am not quite understand... Please try again!")
                # save_img(count)
                break

        # look commands
        elif cmd[0].find('look') != -1:
            # look up
            if cmd[0].find('up') != -1:
                cmd.pop(0)
                event = controller.step("LookUp")
                controller.step("Pass")
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # look down
            elif cmd[0].find('down') != -1:
                cmd.pop(0)
                event = controller.step("LookDown")
                controller.step("Pass")
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # correct input after 'look' but no meaning
            else:
                print("I do not quite understand... Please try again!")
                # save_img(count)
                break

        # rotate / turn commands
        elif cmd[0].find('turn') != -1 or cmd[0].find('rotate') != -1:
            # rotate / turn left
            if cmd[0].find('left') != -1:
                cmd.pop(0)
                event = controller.step("RotateLeft")
                controller.step("Pass")
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # rotate / turn right
            elif cmd[0].find('right') != -1:
                cmd.pop(0)
                event = controller.step("RotateRight")
                controller.step("Pass")
                # save_img(count)
                # show_objectid(numOfObj)
                continue

            # correct input after 'turn' / 'rotate' but no meaning
            else:
                print("I do not quite understand... Please try again!")
                # save_img(count)
                break

        # crouch command
        elif cmd[0].find('crouch') != -1:
            cmd.pop(0)
            event = controller.step(action="Crouch")
            controller.step("Pass")
            # save_img(count)
            # show_objectid(numOfObj)
            continue

        # stand command
        elif cmd[0].find('stand') != -1:
            cmd.pop(0)
            event = controller.step(action="Stand")
            controller.step("Pass")
            # save_img(count)
            # show_objectid(numOfObj)
            continue

        # pick up command
        elif cmd[0].find("pick up") != -1:
            if cmd[0].find('the') != -1:
                start = cmd[0].find('the') + 4
            else:
                start = cmd[0].find('up') + 3

            event = controller.last_event
            n = 0
            is_understandable = False

            distance_dict = {}
            tolerance = 3
            for obj in obj_list:
                distance = lev(cmd[0][start:len(cmd[0])], obj)
                if distance <= tolerance:
                    distance_dict.update({obj: distance})

            if len(distance_dict) != 0:
                distance_dict = dict(sorted(distance_dict.items(), key=lambda x: x[1]))
                suggestion = list(distance_dict.keys())

                cmd[0] = cmd[0].replace(cmd[0][start:len(cmd[0])], suggestion[0])

            while n < numOfObj:
                obj_type = event.metadata["objects"][n]["objectType"]
                obj_id = event.metadata["objects"][n]["objectId"]
                is_visible = event.metadata["objects"][n]["visible"]
                is_pickupable = event.metadata["objects"][n]["pickupable"]

                if obj_type.lower() != 'box' and cmd[0].find(obj_type.lower(), start) != -1:
                    if is_visible is True and is_pickupable is True:
                        cmd.pop(0)
                        event = pickup(obj_id)
                        is_understandable = True
                        break
                    else:
                        cmd.pop(0)
                        print("Oops! " + obj_type + " cannot be picked up!")
                        is_understandable = True
                        break
                n += 1
            if is_understandable is False:
                print("I do not quite understand... Please try again!")
                break

            # save_img(count)
            controller.step("Pass")
            continue

        # put down command
        elif cmd[0].find('put') != -1:
            is_putdownable = False
            if cmd[0].find('down') != -1:
                if cmd[0].find('the') != -1:
                    start = cmd[0].find('the') + 4
                else:
                    start = cmd[0].find('down') + 5

                event = controller.last_event
                n = 0
                hold_obj = ""

                distance_dict = {}
                tolerance = 3
                for obj in obj_list:
                    distance = lev(cmd[0][start:len(cmd[0])], obj)
                    if distance <= tolerance:
                        distance_dict.update({obj: distance})

                if len(distance_dict) != 0:
                    distance_dict = dict(sorted(distance_dict.items(), key=lambda x: x[1]))
                    suggestion = list(distance_dict.keys())

                    cmd[0] = cmd[0].replace(cmd[0][start:len(cmd[0])], suggestion[0])

                # check if object is being held
                while n < numOfObj:
                    obj_type = event.metadata["objects"][n]["objectType"]
                    is_pickedup = event.metadata["objects"][n]["isPickedUp"]
                    if cmd[0].find(obj_type.lower(), start) != -1 and is_pickedup is True:
                        cmd.pop(0)
                        hold_obj = obj_type
                        break
                    n += 1

                if hold_obj == "":
                    print("Wrong object!")
                    break

                n = 0
                while n < numOfObj:
                    obj_type = event.metadata["objects"][n]["objectType"]
                    obj_id = event.metadata["objects"][n]["objectId"]
                    is_visible = event.metadata["objects"][n]["visible"]
                    is_receptacle = event.metadata["objects"][n]["receptacle"]
                    if obj_type != hold_obj and obj_type != "Floor" and is_visible is True and is_receptacle is True:
                        event = putdown(obj_id)
                        print("The " + hold_obj + " was put on " + obj_type)
                        is_putdownable = True
                        break
                    n += 1

                # no object is visible / receptacle to the robot
                if is_putdownable is False:
                    print("Oops! " + hold_obj + " cannot be put down.")

                # save_img(count)
                controller.step("Pass")
                continue

            elif cmd[0].find("on") != -1:
                if cmd[0].find('put the') != -1:
                    start = cmd[0].find('the') + 4
                else:
                    start = cmd[0].find('put') + 4

                if cmd[0][start:len(cmd[0])].find('the') != -1:
                    end = cmd[0].find('on') - 1
                    second_start = cmd[0].find('on the') + 7
                else:
                    end = cmd[0].find('on') - 1
                    second_start = cmd[0].find('on') + 3

                event = controller.last_event
                n = 0
                hold_obj = ""

                distance_dict = {}
                tolerance = 3
                for obj in obj_list:
                    distance = lev(cmd[0][start:end], obj)
                    if distance <= tolerance:
                        distance_dict.update({obj: distance})

                if len(distance_dict) != 0:
                    distance_dict = dict(sorted(distance_dict.items(), key=lambda x: x[1]))
                    suggestion = list(distance_dict.keys())

                    cmd[0] = cmd[0].replace(cmd[0][start:end], suggestion[0])

                # check if object is being held
                while n < numOfObj:
                    obj_type = event.metadata["objects"][n]["objectType"]
                    is_pickedup = event.metadata["objects"][n]["isPickedUp"]
                    if cmd[0].find(obj_type.lower(), start, end) != -1 and is_pickedup is True:
                        hold_obj = obj_type
                        break
                    n += 1

                if hold_obj == "":
                    print("Wrong object!")
                    break

                n = 0
                target_obj = ""

                distance_dict = {}
                tolerance = 3
                for obj in obj_list:
                    distance = lev(cmd[0][second_start:len(cmd[0])], obj)
                    if distance <= tolerance:
                        distance_dict.update({obj: distance})

                if len(distance_dict) != 0:
                    distance_dict = dict(sorted(distance_dict.items(), key=lambda x: x[1]))
                    suggestion = list(distance_dict.keys())

                    cmd[0] = cmd[0].replace(cmd[0][second_start:len(cmd[0])], suggestion[0])

                while n < numOfObj:
                    obj_type = event.metadata["objects"][n]["objectType"]
                    obj_id = event.metadata["objects"][n]["objectId"]
                    is_receptacle = event.metadata["objects"][n]["receptacle"]
                    is_visible = event.metadata["objects"][n]["visible"]
                    if cmd[0].find(obj_type.lower(), second_start) != -1:
                        cmd.pop(0)
                        target_obj = obj_type
                        if is_visible is True and is_receptacle is True:
                            event = putdown(obj_id)
                            print(hold_obj + " was put on " + target_obj)
                            is_putdownable = True
                            break
                        else:
                            print(hold_obj + " cannot be put down on " + target_obj)
                            break
                    n += 1

                if target_obj == "":
                    print("Wrong object!")
                    break

                if is_putdownable is False:
                    break

            else:
                print("I do not quite understand... Please try again!")
                break

            # save_img(count)
            controller.step("Pass")
            continue

        else:
            cmd.pop(0)
            print("I do not quite understand... Please try again!")
            # save_img(count)


        # query = controller.step(
        #     action="GetObjectInFrame",
        #     x=0.50,
        #     y=0.50,
        #     checkVisible=False
        # )

    # object_id = query.metadata["actionReturn"]
    # print(object_id)
    # controller.step("Pass")
