import os

import cv2
import numpy as np

try:
    from PIL import Image
    from PIL import ImageDraw
    from PIL import ImageFont
except ImportError:  # pragma: no cover - robot image may not have PIL until deps are installed.
    Image = None
    ImageDraw = None
    ImageFont = None


FONT_CANDIDATES = (
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJKkr-Regular.otf',
    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
    '/System/Library/Fonts/Supplemental/AppleGothic.ttf',
    '/Library/Fonts/AppleGothic.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
)

_FONT_PATH = None
_FONT_CACHE = {}


def _find_font_path():
    global _FONT_PATH
    if _FONT_PATH is not None:
        return _FONT_PATH
    for path in FONT_CANDIDATES:
        if os.path.exists(path):
            _FONT_PATH = path
            return _FONT_PATH
    _FONT_PATH = ''
    return _FONT_PATH


def _font(size):
    if ImageFont is None:
        return None
    size = max(int(size), 8)
    cached = _FONT_CACHE.get(size)
    if cached is not None:
        return cached
    font_path = _find_font_path()
    if not font_path:
        return None
    try:
        loaded = ImageFont.truetype(font_path, size)
    except OSError:
        return None
    _FONT_CACHE[size] = loaded
    return loaded


def draw_text_bgr(
    image,
    text,
    origin,
    font_size=18,
    color=(255, 255, 255),
    stroke_color=(0, 0, 0),
    stroke_width=2,
):
    """Draw UTF-8 text into an OpenCV BGR image. Returns False when PIL/font is unavailable."""
    font = _font(font_size)
    if Image is None or ImageDraw is None or font is None:
        return False

    x, y = int(origin[0]), int(origin[1])
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil_image)

    fill = (int(color[2]), int(color[1]), int(color[0]))
    stroke_fill = (int(stroke_color[2]), int(stroke_color[1]), int(stroke_color[0]))
    draw.text(
        (x, y),
        str(text),
        font=font,
        fill=fill,
        stroke_width=max(int(stroke_width), 0),
        stroke_fill=stroke_fill,
    )

    image[:] = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)
    return True
