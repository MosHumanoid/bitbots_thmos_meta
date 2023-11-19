from bitbots_splines.smooth_spline import SmoothSpline

s = SmoothSpline()

s.add_point(0, 0, 0, 0)
s.add_point(1, 1, 0, 0)
s.compute_spline()
print(s.pos(0.5))
