class CintaTransportadora:

    def __init__(self, model, data, segments, pos, seg, numero, vel, dadr):
        self.model = model
        self.data = data
        self.segments = segments
        self.pos = pos
        self.seg = seg
        self.numero = numero
        self.vel = vel
        self.dadr = dadr

    def get_ids(self, model):
        body_ids = []
        dof_addrs = []
        for i in range(5):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"obj{i}")
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            body_ids.append(bid)
            dof_addrs.append(model.jnt_dofadr[jid])
        return body_ids, dof_addrs

    def reset_objects(self, model, data, segments):
        """Reinicia las 5 piezas. El cubo rojo comienza cerca de la isla."""
        starts = [
            (-LANE, 0.95, OBJECT_REST_Z[0], 3),   # cubo rojo: llega pronto a la estación Panda
            ( LANE, -0.72, OBJECT_REST_Z[1], 1),
            ( 0.58,  LANE, OBJECT_REST_Z[2], 2),
            (-0.62, -LANE, OBJECT_REST_Z[3], 0),  # cilindro verde lejos de la estación robot
            (-0.45,  LANE, OBJECT_REST_Z[4], 2),
        ]

        for i, (x, y, z, seg) in enumerate(starts):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            qadr = model.jnt_qposadr[jid]
            dadr = model.jnt_dofadr[jid]

            data.qpos[qadr:qadr+3] = [x, y, z]
            data.qpos[qadr+3:qadr+7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[dadr:dadr+6] = 0.0
            segments[i] = seg

        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(model, data)


    def lock_object_rotations(self, model, data):
        """Mantiene todas las piezas con orientación fija respecto al mundo."""
        if not LOCK_OBJECT_ROTATION:
            return

        for i in range(5):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            qadr = model.jnt_qposadr[jid]
            dadr = model.jnt_dofadr[jid]
            data.qpos[qadr+3:qadr+7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[dadr+3:dadr+6] = 0.0


    def lock_cylinders_to_belt_center(self, model, data, segments):
        """Mantiene los cilindros en la línea central de los tramos rectos."""
        if not LOCK_CYLINDERS_TO_CENTERLINE:
            return

        for i in CYLINDER_OBJECT_IDS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
            qadr = model.jnt_qposadr[jid]
            dadr = model.jnt_dofadr[jid]
            x = float(data.qpos[qadr])
            y = float(data.qpos[qadr + 1])
            seg = segments[i]

            if seg == 0 and x < TURN_START:
                data.qpos[qadr + 1] = -LANE
                data.qvel[dadr + 1] = 0.0
            elif seg == 1 and y < TURN_START:
                data.qpos[qadr] = LANE
                data.qvel[dadr] = 0.0
            elif seg == 2 and x > -TURN_START:
                data.qpos[qadr + 1] = LANE
                data.qvel[dadr + 1] = 0.0
            elif seg == 3 and y > -TURN_START:
                data.qpos[qadr] = -LANE
                data.qvel[dadr] = 0.0
            elif seg == 4:
                data.qpos[qadr + 1] = BRANCH_Y
                data.qvel[dadr + 1] = 0.0


    def update_segment(self, pos, seg):
        """Máquina de estados del circuito. La Cinta 5 solo la activa el robot."""
        x, y = float(pos[0]), float(pos[1])

        if seg == 0 and x >= CORNER_SWITCH:
            return 1
        if seg == 1 and y >= CORNER_SWITCH:
            return 2
        if seg == 2 and x <= -CORNER_SWITCH:
            return 3
        if seg == 3 and y <= -CORNER_SWITCH:
            return 0
        return seg


    def cinta_activa(self, seg):
        """Devuelve la bandera de marcha de la cinta asociada al segmento 0..4."""
        if seg == 0:
            return BANDERA_CINTA_1
        if seg == 1:
            return BANDERA_CINTA_2
        if seg == 2:
            return BANDERA_CINTA_3
        if seg == 3:
            return BANDERA_CINTA_4
        if seg == 4:
            return BANDERA_CINTA_5
        return False


    def estado_cintas(self):
        """Estado actual de las cinco banderas, en orden Cinta 1..5."""
        return [
            BANDERA_CINTA_1,
            BANDERA_CINTA_2,
            BANDERA_CINTA_3,
            BANDERA_CINTA_4,
            BANDERA_CINTA_5,
        ]


    def conmutar_cinta(self, numero):
        """Conmuta en tiempo de ejecución una bandera de marcha/parada."""
        global BANDERA_CINTA_1, BANDERA_CINTA_2, BANDERA_CINTA_3
        global BANDERA_CINTA_4, BANDERA_CINTA_5

        if numero == 1:
            BANDERA_CINTA_1 = not BANDERA_CINTA_1
            return BANDERA_CINTA_1
        if numero == 2:
            BANDERA_CINTA_2 = not BANDERA_CINTA_2
            return BANDERA_CINTA_2
        if numero == 3:
            BANDERA_CINTA_3 = not BANDERA_CINTA_3
            return BANDERA_CINTA_3
        if numero == 4:
            BANDERA_CINTA_4 = not BANDERA_CINTA_4
            return BANDERA_CINTA_4
        if numero == 5:
            BANDERA_CINTA_5 = not BANDERA_CINTA_5
            return BANDERA_CINTA_5
        raise ValueError(f"Número de cinta inválido: {numero}")


    def transport_force(self, pos, vel, seg):
        """Fuerza de transporte de las 4 cintas y de la rama al horno."""
        x, y = float(pos[0]), float(pos[1])
        vx, vy = float(vel[0]), float(vel[1])

        fx = 0.0
        fy = 0.0

    # Si la bandera de esta cinta está a False, se aplica frenado viscoso
    # sobre el plano XY. La física sigue activa y el resto de cintas/robot
    # continúan funcionando de forma independiente.
        if not cinta_activa(seg):
            fx = VEL_KP * (0.0 - vx)
            fy = VEL_KP * (0.0 - vy)
            return (
                float(np.clip(fx, -MAX_FORCE, MAX_FORCE)),
                float(np.clip(fy, -MAX_FORCE, MAX_FORCE)),
            )

        if seg == 0:  # inferior: +X -> +Y
            alpha = np.clip((x - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = BELT_SPEED * (1.0 - 0.45 * alpha)
            desired_vy = BELT_SPEED * (0.85 * alpha)
            fx = VEL_KP * (desired_vx - vx)
            fy = VEL_KP * (desired_vy - vy) + (1.0 - alpha) * CENTER_KP * (-LANE - y)

        elif seg == 1:  # derecha: +Y -> -X
            alpha = np.clip((y - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = -BELT_SPEED * (0.85 * alpha)
            desired_vy = BELT_SPEED * (1.0 - 0.45 * alpha)
            fx = VEL_KP * (desired_vx - vx) + (1.0 - alpha) * CENTER_KP * (LANE - x)
            fy = VEL_KP * (desired_vy - vy)

        elif seg == 2:  # superior: -X -> -Y
            alpha = np.clip((-x - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = -BELT_SPEED * (1.0 - 0.45 * alpha)
            desired_vy = -BELT_SPEED * (0.85 * alpha)
            fx = VEL_KP * (desired_vx - vx)
            fy = VEL_KP * (desired_vy - vy) + (1.0 - alpha) * CENTER_KP * (LANE - y)

        elif seg == 3:  # izquierda: -Y -> +X en esquina inferior
            alpha = np.clip((-y - TURN_START) / max(CORNER_SWITCH - TURN_START, 1e-6), 0.0, 1.0)
            desired_vx = BELT_SPEED * (0.85 * alpha)
            desired_vy = -BELT_SPEED * (1.0 - 0.45 * alpha)
            fx = VEL_KP * (desired_vx - vx) + (1.0 - alpha) * CENTER_KP * (-LANE - x)
            fy = VEL_KP * (desired_vy - vy)

        elif seg == 4:  # Cinta 5 al horno: -X
            desired_vx = -BRANCH_SPEED
            desired_vy = 0.0
            fx = VEL_KP * (desired_vx - vx)
            fy = VEL_KP * (desired_vy - vy) + CENTER_KP * (BRANCH_Y - y)

        return float(np.clip(fx, -MAX_FORCE, MAX_FORCE)), float(np.clip(fy, -MAX_FORCE, MAX_FORCE))



    def send_object_back_to_line(self, model, data, i, dadr, segments):
        """Al entrar en el horno, reaparece en Cinta 1 para mantener flujo continuo."""
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
        qadr = model.jnt_qposadr[jid]
        x0 = -1.05 + 0.18 * (i % 3)
        data.qpos[qadr:qadr+3] = [x0, -LANE, OBJECT_REST_Z[i]]
        data.qpos[qadr+3:qadr+7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[dadr:dadr+6] = 0.0
        segments[i] = 0


    def apply_transport(self, model, data, body_ids, dof_addrs, segments, oven_counts, robot_state):
        """Aplica transporte; el cubo reservado se inmoviliza hasta que lo toma el Panda."""
        data.xfrc_applied[:] = 0.0

        for i, bid in enumerate(body_ids):
            pos = data.xpos[bid]
            dadr = dof_addrs[i]
            vel = data.qvel[dadr:dadr+3]

        # Si el robot ya porta la pieza, la cinta no debe aplicar ninguna fuerza.
            if i == robot_state["grasped_obj"]:
                continue

        # Cubo reservado: mantenerlo centrado y detenido en la zona de pick.
            if i == robot_state["target_obj"] and robot_state["state"] != "idle":
                fx, fy = robot.robot_hold_force(pos, vel, ROBOT_CONFIG, MAX_FORCE)
            else:
                segments[i] = update_segment(pos, segments[i], i)
                fx, fy = transport_force(pos, vel, segments[i])

            data.xfrc_applied[bid, 0] = fx
            data.xfrc_applied[bid, 1] = fy

            if segments[i] == 4 and pos[0] <= OVEN_ENTRY_X:
                oven_counts[i] += 1
                send_object_back_to_line(model, data, i, dadr, segments)
                continue

            if pos[2] < -0.1 or abs(pos[0]) > 5.0 or abs(pos[1]) > 2.4:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"obj{i}_free")
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr:qadr+3] = [-0.8 + 0.25*i, -LANE, OBJECT_REST_Z[i]]
                data.qpos[qadr+3:qadr+7] = [1.0, 0.0, 0.0, 0.0]
                data.qvel[dadr:dadr+6] = 0.0
                segments[i] = 0