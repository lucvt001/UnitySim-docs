## Adding water object, waves and rendering underwater

https://github.com/MARUSimulator/marus-example/wiki/3.-Quick-start

## Manipulating water appearance

There are two areas to modify: Ocean (Material) in the Water object and Depth Fog Density Factor in Crest Underwater Renderer of the Camera.

### Ocean (Material) in the Water object

Relevant parameters: from Exposed Properties onward. Most relevant will be Colour and Transparency.

## Adding background sky

Step 1: Add to scene hierarchy: rendering -> volume -> sky and fog global volume  

Step 2: In the Inspector, Visual Environment, change sky type to HDRI Sky. Untick Physically Based Sky.

Step 3: override -> HDRi Sky. Select the HDRI sky from the dropdown. Make sure that the HDRI sky is in cubemap format. (To do that, select the HDRI sky in the Project window, and in the Inspector, change Texture Shape to Cube and click Apply.)

Step 4: override -> exposure. Change the mode to Automatic. Adjust the exposure in HDRI Sky to higher values if the sky appears dark.