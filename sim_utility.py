import time
import random
import carla as c


def simulation_setup(args):
    client = c.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    world = client.get_world()
    spawn_point = args.spawn  # Select another spawn point here
    actors = []
    # get blueprints
    blueprint_library = world.get_blueprint_library()
    car_bp = random.choice(blueprint_library.filter('vehicle.bmw.*'))
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    obstacle_sensor_bp = blueprint_library.find('sensor.other.obstacle')
    obstacle_sensor_bp.set_attribute('distance', '10')
    # get the spawn points for the map
    spawn_points = world.get_map().get_spawn_points()

    car_transform = spawn_points[spawn_point]
    main_car = world.spawn_actor(car_bp, car_transform)  # spawn the main car
    actors.append(main_car)
    # set the camera slightly behind and facing down toward the main car

    obstacle_loc = c.Location(x=1, y=0, z=0.5)
    obstacle_rotation = c.Rotation(pitch=0)
    obstacle_transform = c.Transform(obstacle_loc, obstacle_rotation)
    obstacle_sensor = world.spawn_actor(obstacle_sensor_bp, obstacle_transform, attach_to=main_car)
    actors.append(obstacle_sensor)

    camera_transform = c.Transform(c.Location(x=-8, y=0, z=5), c.Rotation(pitch=-30))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=main_car)  # spawn camera and attach to car
    world_map = world.get_map()  # get the map of the world for waypoint stuff
    control = c.VehicleControl()  # get control object for main car

    return client, world, actors, main_car, camera, obstacle_sensor, world_map, control
