from Arm import Controller


controller = Controller(
    show_video=True,
    debug=True,
    enable_motion=True
)
controller._open_serial()
controller.home()

try:

    while True:

        angles = controller.update()

        if angles is not None:

            print("ARM ANGLES:", angles)

            # Your arm code here
            # arm.move(angles)


except KeyboardInterrupt:

    pass

finally:

    controller.close()