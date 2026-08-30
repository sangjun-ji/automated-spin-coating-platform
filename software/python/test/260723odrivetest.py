import odrive
from odrive.utils import dump_errors

odrv0 = odrive.find_any()

# 1. 빨간불을 유발한 원인 진단
print("🔻 빨간불이 켜진 원인:")
dump_errors(odrv0)

# 2. 에러 클리어 (빨간불 끄기)
if hasattr(odrv0, 'clear_errors'):
    odrv0.clear_errors()
    print("✨ 에러가 클리어되고 빨간불이 꺼졌습니다!")