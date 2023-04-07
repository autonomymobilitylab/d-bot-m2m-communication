import time

class BuzzerUtil():
    def __init__(self):
        pass

    def get_recent_tags_in_workarea(self, tags, areaid):
        res = []
        for tag in tags:
            if (self.is_in_work_area(tag, areaid) and tag['DeviceId'] not in res):
                res.append(tag['DeviceId'])
        return res

    def is_in_work_area(self, tag, area_id):
        # Timestamp in millis, converting to seconds
        tag_timestamp = tag['Timestamp'] / 1000
        if (area_id in tag['Areas'] and (time.time() - tag_timestamp < 10)):
            return True
        return False