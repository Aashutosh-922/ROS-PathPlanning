import cv2 as cv

img = cv.imread(r'assests/yellow_detect.png', 1)

#img = cv.resize(img, (400,400))
#img = cv.resize(img, (0,0), fx=0.5, fy=0.5)     #changing size

# img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)   rotate image

#cv.imwrite('new_img.png', img) creating a new image

print(img.size)
print(img.shape)

cv.namedWindow('Image')
cv.imshow('Image', img)

cv.waitKey(0)
cv.destroyAllWindows()

