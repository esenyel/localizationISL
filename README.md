# localizationISL
The localization module which works in conjuction with placeDetectionISL and createBDSTISL to estimate the location of the revisited points in an unknown environment.
It listens topics currentPlaceID from placeDetectionISL and recognizedPlaceID from createBDSTISL topic and estimate the locations of the members of current place based on the locations of the base points of recognized place.
