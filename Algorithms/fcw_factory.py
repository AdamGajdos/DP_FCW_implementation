import Algorithms as algo


class FCWFactory:
    def __init__(self):
        pass

    def get_fcw(self, fcw_type: str) -> algo.FCWAlgorithm:
        if fcw_type == 'fcw_algo_ttc_braking_distance':
            return algo.fcw_algo_ttc_braking_distance.FCWAlgorithmTTCBrakingDistance()
        elif fcw_type == 'fcw_algo_ttc_1cam':
            return algo.fcw_algo_ttc_1cam.FCW_Algorithm_TTC_1Cam()
        elif fcw_type == 'fcw_algo_friction':
            return algo.fcw_algo_friction.FCW_Algorithm_Friction()
        elif fcw_type == 'fcw_algo_custom':
            return algo.fcw_algo_custom.FCW_Algorithm_Custom()
        else:
            return None
