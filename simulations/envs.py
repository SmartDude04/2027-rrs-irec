import datetime

from rocketpy import Environment

envURRG = Environment(
    latitude=42.703111,
    longitude=-77.191139,
    elevation=270,
    timezone="America/New_York",
)

tomorrow = datetime.date.today() + datetime.timedelta(days=1)

envURRG.set_date(
    (tomorrow.year, tomorrow.month, tomorrow.day, 12)
)

envURRG.set_atmospheric_model(type="Forecast", file="GFS")

if __name__ == "__main__":
    envURRG.info()
