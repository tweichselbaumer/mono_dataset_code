#pragma once

void makePlane2Img(Eigen::Matrix3f &HK, float * plane2imgX, float * plane2imgY);

void checkIfValid(float * plane2imgX, int idx, float * plane2imgY, Eigen::Vector3f &ppy0, Eigen::Vector2f &a);
