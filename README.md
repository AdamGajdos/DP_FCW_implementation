# DP_FCW_implementation
Implementation of Custom FCW assistant and 2 existing algorithms as refferences.

To run simulation with testing data, run 
```
py main.py
```

If you want to test your own data, add your '.json' file to folder 'SimulationData', and in 'main.py' change variable 'road_data' in method '__prepare_road'.

The structure of your '.json' file should be:
```
{
    "points": [
        {
            "position": {
                "longitude": 21.94089,
                "latitude": 48.97522
            },
            "velocity": 19.776305716302147,
            "acceleration": 0.26975665790229064,
            "deceleration": -0.26975665790229064,
            "delay": 0.273,
            "distance": 96.85492551657218,
            "road_info": {
                "road_type": "ASPHALT",
                "condition": "DRY"
            },
            "steep": "UPHILL",
            "angle": 0
        },...
    ]
}
```

Feel free to edit.
