def shoelace_formula(vertices):
    """
    Calculate the area of an irregular polygon using the Shoelace method.

    Parameters:
    vertices (list of tuples): A list of (x, y) coordinates of the polygon's vertices
                               in the order in which they are connected.

    Returns:
    float: The area of the polygon.
    """

    n = len(vertices)
    if n < 3:
        raise ValueError("A polygon must have at least 3 vertices")

    area = 0

    # Shoelace formula
    for i in range(n-1):
        x1, y1 = vertices[i]
        print(x1,y1)
        x2, y2 = vertices[(i + 1) % n]
        print(x2, y2)
        area += x1 * y2 - x2 * y1
        print(area)

    area = abs(area) / 2.0
    return area

# Example usage:
vertices = [(4, 10), (9, 7), (11, 2), (2, 2)]
area = shoelace_formula(vertices)
print(f"The area of the polygon is: {area}")
