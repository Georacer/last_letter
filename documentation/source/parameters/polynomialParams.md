## Ponynomial object parameters

In various instances in `last_letter`, natural processes are approximated with polynomials of one or more viariables (dimensions). Hence, polynomial objects are built and evaluated when needed.
In order to specify the polynomial type and coefficients, parameters from the parameter server are read by the simulation during initialization. These parameters are explained below:

`polyType`:(**integer**) The type of polynomial to be created.
- 0: 1D polynomial. A polynomial of 1 variable
- 1: 2D polynomial. A polynomial of 2 variables
- 2: A cubic spline.

## 1D polynomial parameters
If a 1D polynomial is selected, the following parameters are required for its construction:
`polyNo`: The order of the polynomial.

`coeffs`: The list containing the polynomial coefficients, starting from the zero-order term. This list should have `polyNo+1` elements.
For example, for a 3rd order 1D polynomial, the corresponding field should be:
`myPoly/coeffs: [0.0, 2.5, -0.8, -2.6]`

## 2D polynomial parameters
If a 2D polynomial is selected, the following parameters are required for its construction:

`polyNo`:(**integer**) The list with the two orders of the polynomial `n1` and `n2`, one for each variable, `v1` and `v2`. The order of `v1` should be less than the order of `v2`, otherwise the input variables should be swapped in the code.
Example specification: `myPoly/polyNo: [1, 5]`

`coeffs`: The list containing the polynomial coefficients, starting from the zero-order term. The coefficients then start from the one corresponding to the term `v1^0*v2^1`, then the term `v1^1*v2^1` etc, up to the term `v1^n1*v2^0`. This list should have `2*n2 + 2*n1*n2 + n1 - n1*n1 + 2)/2` elements.
For example, for `n1=1` and `n2=5`, the corresponding field should be:
`myPoly/coeffs:`
`[7.554e+4, -1076.0, 5.768, -0.01416, 1.66e-5, -7.553e-9,`
`1.676e+5, -1487.0, 4.831, -0.006781, 3.493e-6]`

## Cubic spline
If a cubic spline is selected, the following parameters are required for its construction:
`breaksNo`:(**integer**) The number of transition points contained in the spline.

`breaks`: The values of the transition points. The length of this list should be `breaksNo+1` with the first element acting as a lower bound for your expected range.

`coeffs`: The list (1D array) holding the cubic spline coefficients for each spline section. Each row has four elements, starting from the zero-order term up to the 3rd order term. This array has `4*breaksNo` elements. However, since .yaml files only accept list inputs, all rows are concatenated and inserted serially, separated with commas.
For example, for a spline with 3 transition points, the corresponding field would be:
`myPoly/coeffs:`
`[0.1, 0.2, 0.3, 0.4,`
`0.15, 0.25, 0.24, 0.45,`
`0.28, 0.28, 0.38 0.48]`

[back to table of contents](../../../README.md)