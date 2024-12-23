import cv2
import numpy as np

def determine_text_size(height):
    """Takes height as input and changes it by a fraction (fraction_val)
    Returns the rescaled height
    """
    fraction_val = 2 / 2000
    new_text_ht = height * fraction_val
    # if new_text_ht < 1:
    #     new_text_ht = 1
    return new_text_ht


def put_texts(
    img,
    test_tuple_list=None,
    txt_thickness=3,
    v_space=50,
    txt_color=(0, 255, 0),
    default_align=None,
    offsetval=0,
    font=cv2.FONT_HERSHEY_COMPLEX,
    draw_bg=True,
    bg_color=(0, 0, 0),
):
    """Given an image(img) and a list of texts(test_tuple_list),
    it displays the text on the image in the given position.
    Returns:
    img: image with text
    """
    if test_tuple_list is None:
        test_tuple_list = []
    ht, wd = img.shape[:2]
    l_ct = 1
    r_ct = 1
    align_left = None
    text_height = determine_text_size(ht)
    side_margin = 50

    # if not len(test_tuple_list):
    if len(test_tuple_list) == 0:
        return img

    for index, txt_tuple in enumerate(test_tuple_list):
        if isinstance(txt_tuple, str):
            text = txt_tuple
            if default_align is None:
                align_left = True
            if txt_color is None:
                txt_color = (255, 255, 255)

        elif isinstance(txt_tuple, bool):
            text = f"Oclusion {txt_tuple}"

        elif len(txt_tuple) == 3:
            text, txt_color, align_left = txt_tuple

        elif len(txt_tuple) == 0:
            break

        else:
            text = txt_tuple[0]
            if default_align is None:
                align_left = True
            if txt_color is None:
                txt_color = (255, 255, 255)
        text_size = cv2.getTextSize(
            text, fontFace=font, fontScale=text_height, thickness=txt_thickness
        )

        if align_left:
            y_ = v_space * (l_ct) + text_height
            if offsetval:
                y_ += int(offsetval[1])
                left_gap = int(offsetval[0])
            else:
                left_gap = side_margin
            l_ct += 1
        else:
            y_ = v_space * (r_ct) + text_height
            if offsetval:
                y_ += int(offsetval[1])
                left_gap = int(offsetval[0])
            else:
                left_gap = wd - text_size[0][0] - side_margin
            r_ct += 1
        put_text(
            text,
            img,
            left_gap,
            int(y_),
            txt_color,
            text_height,
            txt_thickness,
            font=font,
            draw_bg=draw_bg,
            bg_color=bg_color,
        )
    return img


def put_text(
    text,
    image,
    x,
    y,
    color=(255, 255, 255),
    font_scale=1,
    thickness=1,
    font=None,
    draw_bg=False,
    bg_color=(0, 0, 0),
    auto_align_h=True,
    auto_align_v=True,
):
    """Puts text on image. Given an image and the input text,
     it is displayed in the input position provided
    Args:
        text (str): text to put on image
        image (numpy.ndarray): input image
        x (int): x coordinate of text
        y (int): y coordinate of text
        color (tuple): color of text
        font_scale (float): font size of text
        thickness (int): thickness of text
        font (str): font of text
        draw_bg (bool): draw background or not
        bg_color (tuple): background color
    Returns:
    image: image with text
    """
    if font is None:
        font = cv2.FONT_HERSHEY_SIMPLEX
    (label_width, label_height), baseline = cv2.getTextSize(
        text, font, font_scale, thickness
    )
    label_width, label_height = int(label_width), int(label_height)
    h, w = image.shape[:2]

    if auto_align_h:  # Adjust text to ensure it's enclosd within image
        if x + label_width > w:
            x = w - label_width
    if auto_align_v:
        if y + label_height > h:
            y = h - label_height

    if draw_bg:
        assert bg_color is not None, "bg_color should be given for draw bg"
        image = cv2.rectangle(
            image,
            (x, max(0, y - label_height)),
            (x + label_width, y + (label_height - baseline)),
            bg_color,
            -1,
        )
    image = cv2.putText(
        image, text, (int(x), int(y)), font, font_scale, color, thickness, cv2.LINE_AA
    )
    return image


def get_random_colors():
    seed = np.random.seed(42)
    R = np.random.randint(0, 255, (5000, 1))
    G = np.random.randint(50, 255, (5000, 1))
    B = np.random.randint(50, 255, (5000, 1))
    colors = np.concatenate((R,G,B), axis=1)
    colors = colors/255
    return colors