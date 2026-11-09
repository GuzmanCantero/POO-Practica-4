from horno import Horno
from mujoco import MuJoCo
import robot


class PlantaIndustrial:

    def __init__(self):
        self.cintas = []
        self.camaras = []
        self.objetos = []
        self.horno = Horno(5, 1)
        self.mujoco = MuJoCo
        self.robot = src.robot

    def inicializar(self):

        viewer_ctx = None
        if SHOW_MUJOCO_VIEWER:
            viewer_ctx = mujoco.viewer.launch_passive(model, data)
            viewer_ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer_ctx.cam.fixedcamid = overview_id



        overview_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "overview"
        )

        viewer_ctx = None

        if SHOW_MUJOCO_VIEWER:
            viewer_ctx = mujoco.viewer.launch_passive(model, data)
            viewer_ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer_ctx.cam.fixedcamid = overview_id










        model = robot.build_model_with_franka(build_mjcf(), ROBOT_CONFIG)
        data = mujoco.MjData(model)

        overview_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    
        body_ids, dof_addrs = get_ids(model)
        robot_ids = robot.get_robot_ids(model)
        robot_state = robot.new_robot_state()
        segments = [3, 1, 2, 3, 0]
        oven_counts = [0, 0, 0, 0, 0]

        reset_objects(model, data, segments)
        robot.initialize_robot(model, data, robot_ids, robot_state)

    def actualizar(self):

        if ROBOT_IK_TEST_MODE:
            robot.set_ik_target_rpy(
                model, robot_ids, robot_state,
                ROBOT_IK_TARGET_POSITION,
                ROBOT_IK_TARGET_RPY_DEG,
                orientation_frame=ROBOT_IK_ORIENTATION_FRAME,
            )
            mujoco.mj_forward(model, data)

        initialize_belt_rollers(model, data)

        renderer = None
        if SHOW_CAMERA_MOSAIC:
            renderer = mujoco.Renderer(model, height=CAM_H, width=CAM_W)
            cv2.namedWindow("Camaras de la planta", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camaras de la planta", CAM_W*3, CAM_H*2)

        paused = False
        running = True
        last_cam_time = -1.0

    #el run NO esta permitido y tengo que dividirlo
    def ejecutar(self):
        model = robot.build_model_with_franka(build_mjcf(), ROBOT_CONFIG)
        data = mujoco.MjData(model)
 
        overview_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    
        body_ids, dof_addrs = get_ids(model)
        robot_ids = robot.get_robot_ids(model)
        robot_state = robot.new_robot_state()
        segments = [3, 1, 2, 3, 0]
        oven_counts = [0, 0, 0, 0, 0]

        reset_objects(model, data, segments)
        robot.initialize_robot(model, data, robot_ids, robot_state)

        if ROBOT_IK_TEST_MODE:
            robot.set_ik_target_rpy(
                model, robot_ids, robot_state,
                ROBOT_IK_TARGET_POSITION,
                ROBOT_IK_TARGET_RPY_DEG,
                orientation_frame=ROBOT_IK_ORIENTATION_FRAME,
            )
            mujoco.mj_forward(model, data)

        initialize_belt_rollers(model, data)

        renderer = None
        if SHOW_CAMERA_MOSAIC:
            renderer = mujoco.Renderer(model, height=CAM_H, width=CAM_W)
            cv2.namedWindow("Camaras de la planta", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camaras de la planta", CAM_W*3, CAM_H*2)

        paused = False
        running = True
        last_cam_time = -1.0

        print("\n=== PLANTA ROBOTIZADA: 4 CINTAS + PANDA + HORNO ===")
        print("Circuito: inferior -> derecha -> superior -> izquierda -> inferior")
        print("Estacion robot: Cinta 4 -> Franka Panda -> Cinta 5 -> horno")
        print(f"Piezas que recoge el Panda: {sorted(ROBOT_CONFIG['pick_object_ids'])}")
        print("El desvio dinamico anterior se ha eliminado para evitar el atasco en la interseccion.")
        print("Q/ESC: salir | P: pausa global | 1..5: ON/OFF cintas | R: reset | S: captura")
        if ROBOT_IK_TEST_MODE:
            print("Modo IK TEST: T=pose objetivo configurada | H=pose HOME cartesiana")
            print("Target IK:", robot.ik_target_text(robot_state))
        else:
            print("Modo robot: pick-and-place automático")
        print("Banderas iniciales:", " ".join(
            f"C{i+1}={'ON' if v else 'OFF'}" for i, v in enumerate(estado_cintas())
        ))
        print(f"Camaras: FOV={CAM_FOVY:.0f} deg, altura={CAM_HEIGHT:.2f} m")
        print(f"Bloqueo de rotacion de objetos: {LOCK_OBJECT_ROTATION}")
        print(f"Cilindros centrados: {LOCK_CYLINDERS_TO_CENTERLINE}")
        print(f"Panda base=({ROBOT_CONFIG['base_x']:.2f}, {ROBOT_CONFIG['base_y']:.2f}, {ROBOT_CONFIG['base_z']:.2f}) yaw={ROBOT_CONFIG['base_yaw_deg']:.0f} deg")
        print("Programacion procedural: no se usan clases.\n")

        viewer_ctx = None
        if SHOW_MUJOCO_VIEWER:
            viewer_ctx = mujoco.viewer.launch_passive(model, data)
            viewer_ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer_ctx.cam.fixedcamid = overview_id

        try:
            while running:
                if viewer_ctx is not None and not viewer_ctx.is_running():
                    break

                step_start = time.perf_counter()

                if not paused:
                    if ROBOT_IK_TEST_MODE:
                        # Modo de validación: se ignora temporalmente la máquina de estados
                        # pick-and-place y se manda directamente una pose 6D al TCP.
                        apply_transport(
                            model, data, body_ids, dof_addrs, segments, oven_counts, robot_state
                        )
                        robot.control_manual_ik(
                            model, data, robot_ids, robot_state,
                            gripper_open=ROBOT_IK_GRIPPER_OPEN,
                        )
                    else:
                        # Modo automático original: Cinta 4 -> Panda -> Cinta 5.
                        robot.update_robot_state_machine(
                            model, data, body_ids, robot_ids, robot_state, segments,
                            OBJECT_REST_Z, ROBOT_CONFIG
                        )
                        apply_transport(
                            model, data, body_ids, dof_addrs, segments, oven_counts, robot_state
                        )
                        robot.control_robot_ik(model, data, robot_ids, robot_state, ROBOT_CONFIG)
                        robot.follow_tcp_with_grasped_object(model, data, robot_ids, robot_state)

                    # Física.
                    mujoco.mj_step(model, data)

                    # Estabilización de piezas.
                    lock_object_rotations(model, data)
                    lock_cylinders_to_belt_center(model, data, segments)
                    if not ROBOT_IK_TEST_MODE:
                        robot.follow_tcp_with_grasped_object(model, data, robot_ids, robot_state)
                    mujoco.mj_forward(model, data)

                if viewer_ctx is not None:
                    viewer_ctx.sync()

                if SHOW_CAMERA_MOSAIC and renderer is not None:
                    if last_cam_time < 0.0 or (data.time - last_cam_time) >= (1.0 / CAM_FPS) or paused:
                        mosaic = render_camera_mosaic(
                            renderer, data, paused, data.time, sum(oven_counts), robot_state
                        )
                        cv2.imshow("Camaras de la planta", mosaic)
                        last_cam_time = data.time

                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord('q'), 27):
                        running = False
                    elif key == ord('p'):
                        paused = not paused
                        print("PAUSA GLOBAL" if paused else "SIMULACIÓN REANUDADA")
                    elif key in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
                        numero_cinta = key - ord('0')
                        activa = conmutar_cinta(numero_cinta)
                        initialize_belt_rollers(model, data)
                        print(f"Cinta {numero_cinta}: {'ON' if activa else 'OFF'}")
                    elif key == ord('r'):
                        reset_objects(model, data, segments)
                        oven_counts[:] = [0, 0, 0, 0, 0]
                        robot_state.clear()
                        robot_state.update(robot.new_robot_state())
                        robot.initialize_robot(model, data, robot_ids, robot_state)
                        if ROBOT_IK_TEST_MODE:
                            robot.set_ik_target_rpy(
                                model, robot_ids, robot_state,
                                ROBOT_IK_TARGET_POSITION, ROBOT_IK_TARGET_RPY_DEG,
                                orientation_frame=ROBOT_IK_ORIENTATION_FRAME,
                            )
                            mujoco.mj_forward(model, data)
                        initialize_belt_rollers(model, data)
                        print("Planta y Panda reiniciados")
                    elif key == ord('t') and ROBOT_IK_TEST_MODE:
                        robot.set_ik_target_rpy(
                            model, robot_ids, robot_state,
                            ROBOT_IK_TARGET_POSITION, ROBOT_IK_TARGET_RPY_DEG,
                            orientation_frame=ROBOT_IK_ORIENTATION_FRAME,
                        )
                        mujoco.mj_forward(model, data)
                        print("[IK] Target configurado:", robot.ik_target_text(robot_state))
                    elif key == ord('h') and ROBOT_IK_TEST_MODE:
                        robot.set_ik_target_home(model, robot_ids, robot_state)
                        mujoco.mj_forward(model, data)
                        print("[IK] Target HOME cartesiano activado")
                    elif key == ord('s'):
                        if 'mosaic' in locals():
                            filename = OUTPUT_DIR / f"mosaico_{int(time.time())}.png"
                            cv2.imwrite(str(filename), mosaic)
                            print(f"Captura guardada: {filename}")

                elapsed = time.perf_counter() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            if viewer_ctx is not None:
                viewer_ctx.close()
            if renderer is not None:
                renderer.close()
            cv2.destroyAllWindows()


#    if __name__ == "__main__":
#        run(self)


