import unittest
import math
from types import SimpleNamespace as S
import numpy as np
from depth_geometry import project_depth, transform_points, filter_points, vision_failure


class GeometryTests(unittest.TestCase):
    def test_invalid_and_out_of_range_depth_never_become_clear_rays(self):
        d=np.array([[0.,np.nan,np.inf,.1,.5,3.]])
        p=project_depth(d,[100.,0,0,0,100.,0,0,0,1],1)
        self.assertEqual(p.shape,(1,3))
        np.testing.assert_allclose(p[0],[.02,0,.5])

    def test_correct_pitch_about_lens_not_chassis_origin(self):
        t=S(rotation=S(x=0,y=0,z=0,w=1),translation=S(x=.1,y=0,z=.077))
        p=transform_points(np.array([[1.,0,0]]),t,.073,-5.)
        np.testing.assert_allclose(p[0],[.1+math.cos(math.radians(5)),0,.15+math.sin(math.radians(5))])

    def test_ground_chassis_and_invalid_points_are_not_marked(self):
        p=np.array([[.8,0,0],[.8,0,.015],[.8,0,.055],[.801,0,.055],[.802,0,.055],[.1,0,.055],
                    [.8,0,.8],[np.nan,0,.1],[3,0,.1]])
        obs,clear=filter_points(p)
        np.testing.assert_allclose(obs,[[.8,0,.055]])
        self.assertEqual(len(clear),5)

    def test_empty_input(self):
        a,b=filter_points(np.empty((0,3)))
        self.assertEqual(len(a)+len(b),0)

    def test_sensor_loss_and_unverified_calibration_block_navigation(self):
        good={'enabled':True,'ready':True,'calibration_verified':True}
        self.assertEqual(vision_failure(True,good,.1),'')
        self.assertTrue(vision_failure(True,good,1.))
        self.assertTrue(vision_failure(True,dict(good,calibration_verified=False),.1))
        self.assertTrue(vision_failure(True,dict(good,ready=False),.1))
        self.assertEqual(vision_failure(False,{},100.),'')


if __name__=='__main__':unittest.main()
