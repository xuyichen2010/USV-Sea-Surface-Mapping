We hightly recommend to watch our demo and poster first
Demo: https://www.youtube.com/watch?v=1mxitBsI-Ow
Poster: https://drive.google.com/file/d/10oPQA3EOSzhQ2bQRENJCDh9cXs3bxN_G/view


poster_visual.m is the latest version of the file we used, there are some section commented out since those are for data collection and abandoned functionality.

To run this application, you need a '.bag' file. The command is 

poster_visual('xxxx.bag')

We collected our data use a Intel RealSense D435 Stero Depth Camera.

All .m file is previous versions we implemented for different functionality, but we combine them as we add more functionality.
	speed.m is for plotting speed
	depthFrame.m is for frequency plot generating
	slope_valid.m is for validating slope with covariance and wave front
	

All .mat file is the data we collected
	waveBack waveFront and WaveTop is the data as the whole image
	sectorFront sectorTop and sectorBack is the data as the sectors
	slope is the slopes of covariance and wave front
	freqData is the data collected for 150s video
	
