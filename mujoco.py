class MuJoCo:
    
    def build_mjcf(self):
        """Construye el MJCF como texto. Todo se genera proceduralmente."""


        SHOW_MUJOCO_VIEWER = True
        SHOW_CAMERA_MOSAIC = True

    # Cámaras de inspección: el soporte/poste está a un lado de cada cinta,
    # mientras que el objetivo óptico queda cerca del borde de la banda. Así
    # la imagen contiene esencialmente banda + pieza y resulta adecuada para
    # reconocimiento posterior de color y forma.
        cam_bottom = (0.0, -(LANE + CAM_LENS_SIDE_OFFSET), CAM_HEIGHT)
        cam_right  = (LANE + CAM_LENS_SIDE_OFFSET, 0.0, CAM_HEIGHT)
        cam_top    = (0.0, LANE + CAM_LENS_SIDE_OFFSET, CAM_HEIGHT)

    # La cámara 4 NO queda encima de la banda. Su x está más allá del borde
    # exterior de la Cinta 4 y su y se aleja de la intersección con Cinta 5.
        cam_left = (
            -(LANE + BELT_HALF_WIDTH + CAM_LEFT_HEAD_CLEARANCE),
            CAM_LEFT_INSPECTION_Y,
            CAM_HEIGHT,
        )
        cam_oven   = (BRANCH_CENTER_X, BRANCH_Y - CAM_OVEN_SIDE_OFFSET, CAM_OVEN_HEIGHT)

    # Se apunta al centro de una zona local de inspección de la banda. El
    # pequeño offset hace que la cámara siga estando físicamente a un lado,
    # pero la dirección de visión sea casi cenital.
        target_bottom = (0.0, -LANE, BELT_TOP_Z + 0.015)
        target_right  = (LANE, 0.0, BELT_TOP_Z + 0.015)
        target_top    = (0.0, LANE, BELT_TOP_Z + 0.015)
        target_left   = (-LANE, CAM_LEFT_INSPECTION_Y, BELT_TOP_Z + 0.015)
        target_oven   = (BRANCH_CENTER_X, BRANCH_Y, BELT_TOP_Z + 0.015)

        pole_bottom_y = -(LANE + CAM_POLE_SIDE_OFFSET)
        pole_right_x  =  (LANE + CAM_POLE_SIDE_OFFSET)
        pole_top_y    =  (LANE + CAM_POLE_SIDE_OFFSET)
        pole_left_x   = -(LANE + BELT_HALF_WIDTH + CAM_LEFT_POLE_CLEARANCE)
        pole_left_y   = CAM_LEFT_INSPECTION_Y
        pole_oven_y   = BRANCH_Y - (BRANCH_HALF_WIDTH + 0.52)

        xy_bottom = camera_xyaxes(cam_bottom, target_bottom)
        xy_right  = camera_xyaxes(cam_right, target_right)
        xy_top    = camera_xyaxes(cam_top, target_top)
        xy_left   = camera_xyaxes(cam_left, target_left)
        xy_oven   = camera_xyaxes(cam_oven, target_oven)

        xml = mujoco.MjModel.from_xml_path("C:/Users/roqui/Desktop/cuatrimestre 3/prog objetos/Práctica 1/planta.xml")
        return xml
