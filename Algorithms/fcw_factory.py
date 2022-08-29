import Algorithms as algo


class FCWFactory:
    def __init__(self):
        pass

    def get_fcw(self, fcw_type: str) -> algo.FCWAlgorithm:
        if fcw_type == 'fcw_algo_ttc_braking_distance':
            return algo.fcw_algo_ttc_braking_distance.FCWAlgorithmTTCBrakingDistance()
        elif fcw_type == 'fcw_algo_ttc_1cam':
            return algo.fcw_algo_ttc_1cam.FCWAlgorithmTTC1Cam()
        elif fcw_type == 'fcw_algo_friction':
            return algo.fcw_algo_friction.FCWAlgorithmFriction()
        elif fcw_type == 'fcw_algo_custom':
            return algo.fcw_algo_custom.FCWAlgorithmCustom()
        else:
            return None
