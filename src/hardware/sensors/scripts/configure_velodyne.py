from io import BytesIO
import json
import os
import time
from urllib.parse import urlencode
from urllib.request import urlopen

from ament_index_python.packages import get_package_share_path
from pycurl import Curl
import yaml


def sensor_do(s: Curl, url: str, pf: dict, buf: BytesIO) -> bool:
    s.setopt(s.URL, url)
    s.setopt(s.POSTFIELDS, pf)
    s.setopt(s.WRITEDATA, buf)
    s.perform()
    rcode = s.getinfo(s.RESPONSE_CODE)
    success = rcode in range(200, 207)
    print("%s %s: %d (%s)" % (url, pf, rcode, "OK" if success else "ERROR"))
    return success


Base_URL = "http://192.168.1.201/cgi/"
sensor = Curl()
buffer = BytesIO()

# load settings yaml
onboard_yaml = os.path.join(get_package_share_path("sensors") / "config" / "VLP32C-Onboard.yaml")
with open(onboard_yaml, "r") as f:
    settings = yaml.safe_load(f)

# reset to defaults
# rc = sensor_do(sensor, Base_URL+'reset', urlencode({'data':'reset_system'}), buffer)
# time.sleep(10)

# iterate through settings
for setting in settings:
    val = settings[setting]

    if setting == "fov":
        for fov in setting:
            val = setting[fov]
            rc = sensor_do(sensor, Base_URL + "setting/fov", urlencode({fov: val}), buffer)
    else:
        rc = sensor_do(sensor, Base_URL + "setting", urlencode({setting: val}), buffer)

    if not rc:
        print("Failed to set %s" % setting)
        break
    time.sleep(1)

# save and check status
if rc:
    rc = sensor_do(sensor, Base_URL + "save", urlencode({"data": "submit"}), buffer)
    time.sleep(10)
    response = urlopen(Base_URL + "status.json")
if response:
    status = json.loads(response.read())
    print("Sensor laser is %s, motor rpm is %s" % (status["laser"]["state"], status["motor"]["rpm"]))

sensor.close()
