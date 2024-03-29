#version 330 core

precision highp float;
uniform sampler2D InputTexture;
uniform sampler2D secondTexture;
uniform vec2 resolution;
uniform float correction1, correction2, correction3, correction4, croppedWidth, croppedHeight, xCenter, yCenter;
uniform float pitch, roll, yaw, fovIn, fovOut, x, y, z, blendFront, blendBack;
uniform int inputProjection, outputProjection, gridLines, blendImages, linearBlend;
in vec4 gl_FragCoord;
bool isTransparent = false; // A global flag indicating if the pixel should just set to transparent and return immediately.
const int EQUI = 0;
const int FISHEYE = 1;
const int FLAT = 2;
const int CUBEMAP = 3;
const int GRIDLINES_OFF = 0;
const int GRIDLINES_ON = 1;
float PI = 3.14159265359;
vec2 SET_TO_TRANSPARENT = vec2(-1.0, -1.0);
vec4 TRANSPARENT_PIXEL = vec4(0.0, 0.0, 0.0, 0.0);
bool FISHEYE_RADIAL_CORRECTION = false;
// uniform vec3 InputRotation;
// A transformation matrix rotating about the x axis by th degrees.
mat3 Rx(float th)
{
    return mat3(1, 0, 0,
    0, cos(th), -sin(th),
    0, sin(th), cos(th));
}
// A transformation matrix rotating about the y axis by th degrees.
mat3 Ry(float th)
{
    return mat3(cos(th), 0, sin(th),
    0,    1,    0,
    -sin(th), 0, cos(th));
}
// A transformation matrix rotating about the z axis by th degrees.
mat3 Rz(float th)
{
    return mat3(cos(th), -sin(th), 0,
    sin(th),  cos(th), 0,
    0,         0   , 1);
}
// Rotate a point vector by th.x then th.y then th.z, and return the rotated point.
vec3 rotatePoint(vec3 p, vec3 th)
{
    return Rx(th.x) * Ry(th.y) * Rz(th.z) * p;
}
// Convert a 3D point on the unit sphere into latitude and longitude.
// In more mathy terms we're converting from "Cartesian Coordinates" to "Spherical Coordinates"
vec2 pointToLatLon(vec3 point)
{
    float r = distance(vec3(0.0, 0.0, 0.0), point);
    vec2 latLon;
    latLon.x = asin(point.z / r);
    latLon.y = atan(point.x, point.y);
    return latLon;
}
// Convert latitude, longitude into a 3d point on the unit-sphere.
// In more mathy terms we're converting from  "Spherical Coordinates" to "Cartesian Coordinates"
vec3 latLonToPoint(vec2 latLon)
{
    float lat = latLon.x;
    float lon = latLon.y;
    vec3 point;
    point.x = cos(lat) * sin(lon);
    point.y = cos(lat) * cos(lon);
    point.z = sin(lat);
    return point;
}
// Convert pixel coordinates from an Equirectangular image into latitude/longitude coordinates.
vec2 equiUvToLatLon(vec2 local_uv)
{
    return vec2(local_uv.y * PI - PI/2.0,
    local_uv.x * 2.0*PI - PI);
}
// Apply radial correction to a 3D point.
vec3 pointRadialCorrection(vec3 point)
{
    // Rotate the point so that latitude corresponds with the center of the frame.
    // We could do it without rotation, by reimplementing pointToLatLon
    vec3 rotation = vec3(-PI/2.0, 0.0, 0.0);
    point = rotatePoint(point, rotation);
    // Get the calibration parameters
    vec4 fishCorrect = vec4(correction1, correction2, correction3, correction4);
    //fishCorrect.xyzw -= 1.0;
    // Get the longitude
    vec2 latLon = pointToLatLon(point);
    // Get the radius of the point in the xy plane
    float r = distance(point.xy, vec2(0.0, 0.0));
    // Apply the calibration parameters
    r += r * (fishCorrect.x + r * (fishCorrect.y + r * (fishCorrect.z + r * fishCorrect.w)));
    point.x = r * sin(latLon.y);
    point.y = r * cos(latLon.y);
    // Make sure the point is back on the unit sphere
    point = normalize(point);
    return rotatePoint(point, -rotation);
}
// Convert  pixel coordinates from an Fisheye image into latitude/longitude coordinates.
vec2 fisheyeUvToLatLon(vec2 local_uv, float fovOutput)
{

    vec2 pos = 2.0 * local_uv - 1.0;
    // The distance from the source pixel to the center of the image
    float r = distance(vec2(0.0,0.0),pos.xy);
    // Don't bother with pixels outside of the fisheye circle
    if (1.0 < r) {
        isTransparent = true;
        return SET_TO_TRANSPARENT;
    }
    float theta = atan(r, 1.0);
    r = tan(theta/fovOutput);
    vec2 latLon;
    latLon.x = (1.0 - r) * (PI/2.0);
    // Calculate longitude
    latLon.y = PI + atan(-pos.x, pos.y);

    if (latLon.y < 0.0) {
        latLon.y += 2.0*PI;
    }
    vec3 point = latLonToPoint(latLon);
    point = rotatePoint(point, vec3(PI/2.0, 0.0, 0.0));
    latLon = pointToLatLon(point);
    return latLon;
}
// Convert a cubemap uv to a 3d point on a unit cube
vec3 cubemapUvToPoint(vec2 local_uv)
{
    float verticalBoundary = 0.5;
    float leftBoundary  = 1.0/3.0;
    float rightBoundary = 2.0/3.0;
    // Position of the source pixel in uv coordinates in the range [-1,1]
    vec2 pos = (2.0 * local_uv) - 1.0;
    vec3 point;
    float faceDistance = fovOut / 3.0;
    // Is it a standard cubemap or an EAC?
    // Link for more details: https://blog.google/products/google-ar-vr/bringing-pixels-front-and-center-vr-video/
    bool equiAngularCubemap = true;
    // Remove overlap in the image.
    float verticalCorrection = 2.0/3.0;
    // No idea why this was needed, but ~1.15 seems to work and pi / e is really close.
    float piDividedByE = 1.155727349790921717910093183312696299120851023164415820499;
    // The faces of the cubemap. To explain I'll define the following:
    // Let's call +X: "Right"
    //            -X: "Left"
    //            +Y: "Forward"
    //            -Y: "Back"
    //            +Z: "Up"
    //            -Z: "Down"
    // Top left face in output image
    if (local_uv.x <= leftBoundary && verticalBoundary <= local_uv.y) {
        pos += vec2(2.0/3.0, -0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Left" face of cube
        point = vec3(-faceDistance, pos.x, verticalCorrection*pos.y);
    }
    // Top Middle Face in output image
    else if (leftBoundary < local_uv.x && local_uv.x <= rightBoundary && verticalBoundary <= local_uv.y) {
        pos += vec2(0.0, -0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Front" face of cube
        point = vec3(pos.x, faceDistance, verticalCorrection*pos.y);
    }
    // Top Right Face in output image
    else if (rightBoundary < local_uv.x && verticalBoundary <= local_uv.y) {
        pos += vec2(-2.0/3.0, -0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Right" face of cube
        point = vec3(faceDistance, -pos.x, verticalCorrection*pos.y);
    }
    // Bottom left face in output image
    else if (local_uv.x <= leftBoundary && local_uv.y < verticalBoundary) {
        pos += vec2(2.0/3.0, 0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Top" face of cube
        point = vec3(-pos.y*verticalCorrection, -pos.x, faceDistance);
    }
    // Bottom Middle Face in output image
    else if (leftBoundary < local_uv.x && local_uv.x <= rightBoundary && local_uv.y < verticalBoundary) {
        pos += vec2(0.0, 0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Back" face of cube
        point = vec3(-pos.y*verticalCorrection, -faceDistance, -pos.x);
    }
    // Bottom Right Face in output image
    else if (rightBoundary < local_uv.x && local_uv.y < verticalBoundary) {
        pos += vec2(-2.0/3.0, 0.5);
        if(equiAngularCubemap)  {
            pos = tan(pos*PI/2.0)/2.0;
            pos.x *= piDividedByE;
        }
        // "Bottom" face of cube
        point = vec3(-pos.y*verticalCorrection, pos.x, -faceDistance);
    }
    return point;
}
// Convert a cubemap image to Latitude/Longitude Points
vec2 cubemapUvToLatLon(vec2 local_uv)
{
    return pointToLatLon(cubemapUvToPoint(local_uv));
}

vec2 flatImageUvToLatLon(vec2 local_uv, float fovOutput)
{
    // Position of the source pixel in uv coordinates in the range [-1,1]
    vec2 pos = 2.0 * local_uv - 1.0;
    float aspectRatio = resolution.x / resolution.y;
    vec3 point = vec3(pos.x*aspectRatio, 1.0/fovOutput, pos.y);
    return pointToLatLon(point);
}

// Convert latitude, longitude into a 3d point on the unit-sphere.
vec3 flatLatLonToPoint(vec2 latLon)
{
    vec3 point = latLonToPoint(latLon);
    // Get phi of this point, see polar coordinate system for more details.
    float phi = atan(point.x, -point.y);
    // With phi, calculate the point on the image plane that is also at the angle phi
    point.x = sin(phi) * tan(PI / 2.0 - latLon.x);
    point.y = cos(phi) * tan(PI / 2.0 - latLon.x);
    point.z = 1.0;
    return point;
}
// Convert latitude, longitude to x, y pixel coordinates on an equirectangular image.
vec2 latLonToEquiUv(vec2 latLon)
{
    vec2 local_uv;
    local_uv.x = (latLon.y + PI)/(2.0*PI);
    local_uv.y = (latLon.x + PI/2.0)/PI;
    // Set to transparent if out of bounds
    if (local_uv.x < -1.0 || local_uv.y < -1.0 || local_uv.x > 1.0 || local_uv.y > 1.0) {
        // Return a isTransparent pixel
        isTransparent = true;
        return SET_TO_TRANSPARENT;
    }
    return local_uv;
}

// Convert latitude, longitude to x, y pixel coordinates on the source fisheye image.
vec2 pointToFisheyeUv(vec3 point, float fovInput, vec4 fishCorrect)
{
    point = rotatePoint(point, vec3(-PI/2.0, 0.0, 0.0));
    // Phi and theta are flipped depending on where you read about them.
    float theta = atan(distance(vec2(0.0,0.0),point.xy),point.z);
    // The distance from the source pixel to the center of the image
    float r = (2.0/PI)*(theta/fovInput);

    // phi is the angle of r on the unit circle. See polar coordinates for more details
    float phi = atan(-point.y, point.x);
    // Get the position of the source pixel
    vec2 sourcePixel;
    sourcePixel.x = r * cos(phi);
    sourcePixel.y = r * sin(phi);
    // Normalize the output pixel to be in the range [0,1]
    sourcePixel += 1.0;
    sourcePixel /= 2.0;
    // Don't bother with source pixels outside of the fisheye circle
    if (1.0 < r || sourcePixel.x < 0.0 || sourcePixel.y < 0.0 || sourcePixel.x > 1.0 || sourcePixel.y > 1.0) {
        // Return a isTransparent pixel
        isTransparent = true;
        return SET_TO_TRANSPARENT;
    }
    return sourcePixel;
}

bool outOfFlatBounds(vec2 xy, float lower, float upper)
{
    vec2 lowerBound = vec2(lower, lower);
    vec2 upperBound = vec2(upper, upper);
    return (any(lessThan(xy, lowerBound)) || any(greaterThan(xy, upperBound)));
}
vec2 latLonToFlatUv(vec2 latLon, float fovInput)
{
    vec3 point = rotatePoint(latLonToPoint(latLon), vec3(-PI/2.0, 0.0, 0.0));
    latLon = pointToLatLon(point);
    float aspectRatio = resolution.x/resolution.y;
    vec2 xyOnImagePlane;
    vec3 p;
    if (latLon.x < 0.0)
    {
        isTransparent = true;
        return SET_TO_TRANSPARENT;
    }
    // Derive a 3D point on the plane which correlates with the latitude and longitude in the fisheye image.
    p = flatLatLonToPoint(latLon);
    p.x /= aspectRatio;
    // Control the scale with the user's fov input parameter.
    p.xy *= fovInput;
    // Position of the source pixel in the source image in the range [-1,1]
    xyOnImagePlane = p.xy / 2.0 + 0.5;
    if (outOfFlatBounds(xyOnImagePlane, 0.0, 1.0))
    {
        isTransparent = true;
        return SET_TO_TRANSPARENT;
    }
    return xyOnImagePlane;
}
void main()
{
    vec2 uv = vec2(gl_FragCoord.x / resolution.x, gl_FragCoord.y / resolution.y);
    // Display fisheye gridlines if they're turned on
    if (gridLines == GRIDLINES_ON && outputProjection == FISHEYE)
    {
        vec2 gridlineUv = uv;
        gridlineUv.x = (gridlineUv.x * resolution.x/resolution.y) - 0.5;
        if (abs(distance(vec2(0.0, 0.0), 2.0 * gridlineUv - 1.0) - 1.0) < 0.01)
        {
            gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
            return;
        }
    }
    vec3 InputRotation = vec3(pitch/180.0, roll/180.0, yaw/180.0);
    vec4 fragColor = vec4(0.0, 0.0, 0.0, 0.0);
    vec4 centerFragColor = vec4(0.0, 0.0, 0.0, 0.0);
    float fovInput = fovIn / 180.0;
    float fovOutput = fovOut / 180.0;
    vec4 fishCorrect = vec4(correction1, correction2, correction3, correction4);
    //fishCorrect.xyzw -= 1.0;
    float lineCount = 0.0;
    float opacity = 1.0;
    // Level Of Detail: how fast should this run?
    // Set LOD to 0 to run fast, set to 2 to blur the image, reducing jagged edges
    const int LOD = 0;
    //TODO Make Antialiasing a little smarter than this.
    //for(int i = -LOD; i <= LOD; i++)
    //{
    //    for(int j = -LOD; j <= LOD; j++)
    //    {
            isTransparent = false;
            vec2 uv_aa = uv;// + vec2(i, j)/vec2(resolution.x, resolution.y);
            // Given some pixel (uv), find the latitude and longitude of that pixel
            vec2 latLon;
            if (outputProjection == EQUI)
              latLon = equiUvToLatLon(uv_aa);
            else if(outputProjection == FISHEYE)
            {
                uv_aa.x = (uv_aa.x * resolution.x / resolution.y) - 0.5;
                latLon = fisheyeUvToLatLon(uv_aa, fovOutput);
            }
            else if (outputProjection == FLAT)
              latLon = flatImageUvToLatLon(uv_aa, fovOutput);
            else if (outputProjection == CUBEMAP)
              latLon = cubemapUvToLatLon(uv_aa);
            // If a pixel is out of bounds, set it to be transparent
            if (isTransparent)
            {
                //continue;
                gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
                return;
            }
            // Create a point on the unit-sphere from the calculated latitude and longitude
            // This sphere uses a right-handed coordinate system
            // X increases from left to right [-1 to 1]
            // Y increases from back to front [-1 to 1]
            // Z increases from bottom to top [-1 to 1]
            vec3 point = latLonToPoint(latLon);
            point = pointRadialCorrection(point);
            // X, Y, Z translation inputs from the user.
            vec3 translation = 5.0*(vec3(x, y, z) - 1.0);
            // Rotate the point based on the user input in radians
            point.xyz += translation;
            point = rotatePoint(point, InputRotation.rgb * PI);
            //gl_FragColor = vec4(point, 1.0);
            //return;
            if (distance(vec3(0.0, 0.0, 0.0), translation) > 1.0 && distance(vec3(0.0, 0.0, 0.0), point) > distance(vec3(0.0, 0.0, 0.0), translation))
            {
                isTransparent = true;
                //continue;
                gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
                return;
            }
            // Convert back to latitude and longitude
            latLon = pointToLatLon(point);
            //gl_FragColor = vec4(latLon.x, latLon.y, 0.0, 1.0);
            //return;
            // if (1.0 < distance(point, vec3(0.0, 0.0, 0.0)))
            // {
            //   // isTransparent == true;
            //   gl_FragColor = vec4(latLon.x, latLon.y, 0.0, 1.0);
            //   return;
            // }
            // Convert back to the normalized pixel coordinate
            vec2 sourcePixel;
            if (inputProjection == EQUI)
            sourcePixel = latLonToEquiUv(latLon);
            else if (inputProjection == FISHEYE)
            sourcePixel = pointToFisheyeUv(point, fovInput, fishCorrect);
            else if (inputProjection == FLAT)
            sourcePixel = latLonToFlatUv(latLon, fovInput);
            vec2 croppedUv = 2.0*sourcePixel-1.0;
            croppedUv = vec2(croppedUv.x * croppedWidth/resolution.x, croppedUv.y * croppedHeight/resolution.y);
            croppedUv = 0.5*croppedUv+0.5;
            croppedUv.x += xCenter/resolution.x - 0.5;
            croppedUv.y += yCenter/resolution.y - 0.5;
            //if (croppedUv.x < 0.0  || croppedUv.y < 0.0 || 1.0 < croppedUv.x || 1.0 < croppedUv.y)
            //{
            //    //continue;
            //    gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
            //    return;
            //}
            //// If a pixel is out of bounds, set it to be transparent
            //else if (isTransparent)
            //{
            //    //continue;
            //    gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
            //    return;
            //}
            if (linearBlend == 1) {
                if (blendFront < point.y) {
                    opacity = 1.0;
                    //gl_FragColor = vec4(0.0, 1.0, 0.0, 1.0);
                    //return;
                }
                else if (blendBack < point.y && point.y < blendFront) {
                    float blendDiff = blendFront - blendBack;
                    float pointDiff = point.y - blendBack;
                    float blendPercent = pointDiff / blendDiff;
                    //gl_FragColor = vec4(1.0-blendPercent, blendPercent, 0.0, 1.0);
                    opacity = blendPercent;
                } 
                else if (point.y < blendBack) { 
                    opacity = 0.0;
                    //gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
                    //return;
                }
            }
            // Set the color of the destination pixel to the color of the source pixel
            vec4 color = texture2D(InputTexture, croppedUv);
            if (blendImages == 1) {
                vec4 t0 = color;
                vec4 t1 = texture2D(secondTexture, uv);
                color = (1.0 - t1.a) * t0 + t1.a * t1;
            }
            if (inputProjection == EQUI && gridLines == GRIDLINES_ON)
            {
                float minDistance = 0.3;
                float lineThickness = minDistance;
                for (float i = -18.0; i <= 18.0; i += 1.0)
                {
                    float distanceToLine = abs(degrees(latLon.y) - i*10.0);
                    if (distanceToLine <= minDistance)
                    minDistance = distanceToLine;
                    distanceToLine = abs(degrees(latLon.x) - i*10.0);
                    if (distanceToLine <= minDistance)
                    minDistance = distanceToLine;
                }
                if (minDistance < lineThickness)
                {
                    color = vec4(0.0, 0.0, 0.0, 1.0);
                    lineCount += 1.0;
                }
            }
            //fragColor += color;
            gl_FragColor = sqrt(color);
            gl_FragColor.w *= opacity;
            //if (i == 0 && j == 0)
            //{
            //    // This is the aliased pixel. If we didn't do antialiasing this is the pixel we'd get.
            //    centerFragColor = color;
            //}
    //    }
    //}
    // antiAliasCount: how many pixels the above loop should have calculated
//    float antiAliasCount = float((1+2*LOD)*(1+2*LOD));
    // If the pixel has any transparency (i.e. the sourcePixel is at the perimeter of the image) then do antialiasing
//    if (fragColor.a < antiAliasCount || lineCount > 0.0)
//    {
//        // Apply antialiasing. Remove the if/else statement if you want to antialias the whole image.
//        gl_FragColor = fragColor / antiAliasCount;
//
//    }
//    else
//    {
//        // Ignore antialiasing
//        gl_FragColor = centerFragColor;
//    }
}
