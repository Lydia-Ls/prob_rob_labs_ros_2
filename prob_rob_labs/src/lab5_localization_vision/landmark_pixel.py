def landmark_pixel(corners):
    if len(corners) < 4:
        return None

    xs = [point.x for point in corners]
    ys = [point.y for point in corners]

    x_min = min(xs)
    x_max = max(xs)

    y_min = min(ys)
    y_max = max(ys)

    # height of the landmark in the pixel space
    height_px = y_max - y_min
    # position of the vertical symmetry axis of the landmark in the pixel space
    center_x = (x_min + x_max) / 2.0

    if height_px <= 0:
        return None

    return height_px, center_x