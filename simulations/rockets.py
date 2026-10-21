from rocketpy import Rocket

"""
All values are in meters and kilograms. All values are considered to be true values
based on the OpenRocket drawings (Subscale_10-07-26.ork) unless otherwise noted.
"""

subscale = Rocket(
    radius=0.077724,
    mass=22.0559,
    inertia=(18.3964, 18.3964, 0.0608),                 # Approximate
    power_off_drag="./data/drag_curve.csv",             # Generated in OpenRocket
    power_on_drag="./data/drag_curve.csv",
    center_of_mass_without_motor=1.607185,
    coordinate_system_orientation="tail_to_nose",
)

nose = subscale.add_nose(
    length=0.8382,
    kind="conical",
    position=2.921,
)

fins = subscale.add_free_form_fins(
    n=4,
    shape_points=[(0,0),(0.364185,0.118491),(0.436626,0.114021),(0.379171,0)],
    position=0.4318,
)

main = subscale.add_parachute(
    name="main",
    cd_s=10.0,
    trigger=800,                        # Ejection altitude in meters.
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=1.5,
    height=1.5,
    porosity=0.0432,
)

drogue = subscale.add_parachute(
    name="drogue",
    cd_s=1.0,
    trigger="apogee",                   # Ejection at apogee.
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=0.381,                       # Closer to true value (Assuming I read documentation correctly).
    height=1.5,
    porosity=0.0432,
)

if __name__ == "__main__":
    subscale.draw()
