# Object Localization Package

## Description
This package was designed to keep track of the most recent position of each aruco marker that it saw, no matter which camera saw it. It is passed in a list of subscriptions to listen to, and each subscription keeps track of the markers it has seen (and when) as a different row in the same 2d array. When the list of seen markers is requested through the attached service, this 2d array is parsed through and only the most recent location of each marker is returned.

## Known Limitations
There was some difficulty with using this package, so it was not implemented on the final robot. A few changes could be made to make it easier to work with namely allowing for estimations of marker positions based off of time since last seen. There was not enough time to make these changes so the package was not used on the rover.