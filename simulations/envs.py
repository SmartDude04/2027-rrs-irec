from rocketpy import Environment

envURRG = Environment(
    latitude=42.703111,
    longitude=-77.191139,
    elevation=270,
    date=(2025, 11, 15, 14),        # (YYYY, MM, DD, HR)
    timezone="America/New_York",
)
envURRG.set_atmospheric_model(type="Forecast", file="GFS")

if __name__ == "__main__":
    envURRG.info()
