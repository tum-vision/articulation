# Single parameter passed to or from model fitting
#
# This mechanism allows to flexibly pass parameters to 
# model fitting (and vice versa). Note that these parameters 
# are model-specific: A client may supply additional
# parameters to the model estimator, and, similarly, a estimator
# may add the estimated model parameters to the model message.
# When the model is then evaluated, for example to make predictions
# or to compute the likelihood, the model class can then use
# these parameters.
#
# A parameter has a name, a value, and a type. The type globally
# indicates whether it is a prior parameter (prior to model fitting),
# or a model parameter (found during model fitting, using a maximum-
# likelihood estimator), or a cached evaluation (e.g., the likelihood
# or the BIC are a typical "side"-product of model estimation, and
# can therefore already be cached).
#
# For a list of currently used parameters, see the documentation at
# http://www.ros.org/wiki/articulation_models
#

uint8 PRIOR=0   # indicates a prior model parameter 
                # (e.g., "sigma_position")
uint8 PARAM=1   # indicates a estimated model parameter 
                # (e.g., "rot_radius", the estimated radius)
uint8 EVAL=2    # indicates a cached evaluation of the model, given 
                # the current trajectory
                # (e.g., "loglikelihood", the log likelihood of the
                # data, given the model and its parameters)

string name     # name of the parameter
float64 value   # value of the parameter
uint8 type      # type of the parameter (PRIOR, PARAM, EVAL)

