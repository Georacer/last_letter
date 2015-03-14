## Environment Parameters

This set of parameters controls the behaviour of the environment model, mainly referring to the atmospheric model.

###Barometric Parameters
These values parametrise the atmosphere model. For more information, refer to section *Atmosphere Model* of [`Modeling a Fixed-Wing UAV`](https://github.com/Georacer/uav-modeling).

`rho`: The air density at sea level (in kg/m^3).

`groundTemp`: The temperature at initialization altitude (in degrees Celcius).

`groundPres`: The pressure at initialization altitude (in mBar).

###Statis Wind Parameters
By using these parameters you can control the direction and magnitude of the static wind. The wind shear model is exponential, with the velocity of the wind increasing with altitude, as explained in section `Wind Disturbances` of [`Modeling a Fixed-Wing UAV`](https://github.com/Georacer/uav-modeling).

`windRef`: The velocity of the wind at the reference altitude (in m/s).

`windRefAlt`: The altitude at which `windRef` is measured (in m).

`windDir`: The direction of the wind (in degrees).

`surfSmooth`: Hellmann exponent, regulating how fast wind picks up as altitude rises.

###Wind Gusts Parameters
Wind gusts are modelled in 'last_letter' using Dryden transfer functions. These stochastic transfer functions take in white noise of unit power and shape its powre spectrum to match that of real-world measurements. More documentation, indicative values and references can be found at section `Wind Disturbances` of [`Modeling a Fixed-Wing UAV`](https://github.com/Georacer/uav-modeling).
Gusts

`Dryden/use`: Set to 0 to disable wind gusts. Set to 1 to enable them.

`Dryden/Lu`: Coefficient controlling the horizontal gust wavelength (in m).

`Dryden/sigmau`: Total power of the horizontal gusts (in m/s).

`Dryden/Lw`: Coefficient controlling the vertical gust wavelength (in m).

`Dryden/sigmaw`: Total power of the vertical gusts (in m/s).

[back to table of contents](../../../README.md)