from rocketpy import SolidMotor

"""
All values are in meters and kilograms. All values are considered to be garbage values
except for where noted otherwise. Further refinement is needed for this section of code.
"""

L2200G = SolidMotor(
    thrust_source="./data/AeroTech_L2200G.eng",
    dry_mass=2.265,                             # True value (assuming I read the data sheet correctly)
    dry_inertia=(0.08833, 0.08833, 0.00159),    # Calculated value (making broad assumptions)
    nozzle_radius=33/1000,
    grain_number=5,
    grain_density=1815,
    grain_outer_radius=33/1000,
    grain_initial_inner_radius=15/1000,
    grain_initial_height=120/1000,
    grain_separation=5/1000,
    grains_center_of_mass_position=0.3405,      # Estimated value (assume cetner of length)
    center_of_dry_mass_position=0.3405,         # Estimated value (not a great estimation though)
    nozzle_position=0,
    burn_time=2.3,                              # True value
    throat_radius=11/1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

L1420R = SolidMotor(
    thrust_source="./data/AeroTech_L1420R.eng",
    dry_mass=2.265,                             # True value (assuming I read the data sheet correctly)
    dry_inertia=(0.08833, 0.08833, 0.00159),    # Calculated value (making broad assumptions)
    nozzle_radius=33/1000,
    grain_number=5,
    grain_density=1815,
    grain_outer_radius=33/1000,
    grain_initial_inner_radius=15/1000,
    grain_initial_height=120/1000,
    grain_separation=5/1000,
    grains_center_of_mass_position=0.3405,      # Estimated value (assume cetner of length)
    center_of_dry_mass_position=0.3405,         # Estimated value (not a great estimation though)
    nozzle_position=0,
    burn_time=2.3,                              # True value
    throat_radius=11/1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

if __name__ == "__main__":
    L2200G.info()
    L1420R.info()
