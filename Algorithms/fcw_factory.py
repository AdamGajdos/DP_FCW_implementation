from Algorithms import fcw_algo, fcw_algo_custom, fcw_algo_ttc_1cam, fcw_algo_ttc_braking_distance, \
    fcw_algo_berkeley, fcw_algo_friction


class FCWFactory:
    def __init__(self):
        pass

    def get_fcw(self, fcw_type: str) -> fcw_algo.FCWAlgorithm:
        if fcw_type == 'fcw_algo_ttc_braking_distance':
            return fcw_algo_ttc_braking_distance.FCWAlgorithmTTCBrakingDistance()
        elif fcw_type == 'fcw_algo_ttc_1cam':
            return fcw_algo_ttc_1cam.FCWAlgorithmTTC1Cam()
        elif fcw_type == 'fcw_algo_custom':
            return fcw_algo_custom.FCWAlgorithmCustom(d_0=10, a_max=8)
        elif fcw_type == 'fcw_algo_berkeley':
            return fcw_algo_berkeley.FCWAlgorithmBerkeley(d_0=10, k1=0.1, k2=0.1, k3=0)  # values chosen randomly - they wasn't provided in document
        elif fcw_type == 'fcw_algo_friction':
            return fcw_algo_friction.FCWAlgoFriction(d_0=5, s='', tau_1='', tau_2='')
        else:
            return None
