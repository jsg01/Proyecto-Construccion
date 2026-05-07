import cv2

img = cv2.imread("picture_taken.jpg")
print(img)
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: x={x}, y={y}")

        # Draw point
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)

        # Draw text
        cv2.putText(
            img,
            f"({x},{y})",
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        cv2.imshow("Image", img)

# Create window
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image', 1920, 1080)
cv2.setMouseCallback("Image", mouse_callback)

# Show image
cv2.imshow("Image", img)

# Wait until ESC pressed
while True:
    key = cv2.waitKey(1)
    if key == 27:  # ESC
        break

cv2.destroyAllWindows()
