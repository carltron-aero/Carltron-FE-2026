# Empirical plane and radial models from the tracking experiments.
import numpy as np
import cv2
PIXEL_CENTRE=np.array([960.,540.])
PIXEL_SCALE=960.
WORLD_CENTRE=np.array([1500.,1500.])
WORLD_SCALE=1500.

def homography(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack([points, np.ones(len(points))]) @ matrix.T
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def linear_seed(world: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    rows, values = [], []
    for (x, y), (u, v) in zip(world, pixels):
        rows.extend([[x, y, 1, 0, 0, 0, -u*x, -u*y],
                     [0, 0, 0, x, y, 1, -v*x, -v*y]])
        values.extend([u, v])
    return np.linalg.lstsq(np.asarray(rows), np.asarray(values), rcond=None)[0]


def matrix_of(parameters: np.ndarray) -> np.ndarray:
    return np.append(parameters[:8], 1).reshape(3, 3)


def project(world: np.ndarray, parameters: np.ndarray) -> np.ndarray:
    undistorted = homography(world, matrix_of(parameters))
    radius2 = np.sum(undistorted**2, axis=1)
    factor = np.ones(len(world))
    for order, coefficient in enumerate(parameters[8:], start=1):
        factor += coefficient * radius2**order
    return undistorted * factor[:, None]


def invert(pixels: np.ndarray, parameters: np.ndarray) -> np.ndarray:
    distorted = (pixels - PIXEL_CENTRE) / PIXEL_SCALE
    rd = np.linalg.norm(distorted, axis=1)
    ru = rd.copy()
    for _ in range(30):
        factor = np.ones(len(rd))
        derivative = np.ones(len(rd))
        for order, coefficient in enumerate(parameters[8:], start=1):
            power = ru**(2*order)
            factor += coefficient * power
            derivative += (2*order+1)*coefficient * power
        update = (ru*factor-rd) / derivative
        ru -= update
        if np.max(np.abs(update)) < 1e-12:
            break
    ratio = np.divide(ru, rd, out=np.ones_like(ru), where=rd > 0)
    world = homography(distorted*ratio[:, None], np.linalg.inv(matrix_of(parameters)))
    return world*WORLD_SCALE + WORLD_CENTRE


def valid(parameters: np.ndarray) -> bool:
    if not np.all(np.isfinite(parameters)):
        return False
    r2 = np.linspace(0, 2, 100)
    factor = np.ones_like(r2)
    derivative = np.ones_like(r2)
    for order, coefficient in enumerate(parameters[8:], start=1):
        factor += coefficient*r2**order
        derivative += (2*order+1)*coefficient*r2**order
    return bool(np.min(factor) > .5 and np.min(derivative) > .2)


def fit(world_mm: np.ndarray, pixels: np.ndarray, radial_terms: int) -> np.ndarray:
    world = (world_mm-WORLD_CENTRE)/WORLD_SCALE
    observed = (pixels-PIXEL_CENTRE)/PIXEL_SCALE
    parameters = np.append(linear_seed(world, observed), np.zeros(radial_terms))
    damping = .001
    residual = (project(world, parameters)-observed).ravel()
    for _ in range(160):
        columns = []
        for j in range(len(parameters)):
            delta = np.zeros_like(parameters)
            delta[j] = 1e-6
            columns.append(((project(world, parameters+delta)-observed).ravel()-residual)/1e-6)
        jacobian = np.column_stack(columns)
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        diagonal = np.maximum(np.diag(normal), 1e-8)
        step = np.linalg.solve(normal + damping*np.diag(diagonal), -gradient)
        proposal = parameters + step
        next_residual = (project(world, proposal)-observed).ravel()
        if valid(proposal) and np.sum(next_residual**2) < np.sum(residual**2):
            improvement = np.sum(residual**2)-np.sum(next_residual**2)
            parameters, residual = proposal, next_residual
            damping = max(damping*.3, 1e-10)
            if improvement < 1e-16 or np.max(np.abs(step)) < 1e-9:
                break
        else:
            damping *= 10
            if damping > 1e12:
                break
    return parameters


def metric(predicted: np.ndarray, known: np.ndarray) -> dict:
    error = np.linalg.norm(predicted-known, axis=1)
    return {'rms_mm': float(np.sqrt(np.mean(error**2))),
            'max_mm': float(np.max(error)),
            'centre_error_mm': float(np.linalg.norm(np.mean(predicted-known, axis=0)))}


def self_test() -> None:
    generator = np.random.default_rng(5)
    world = generator.uniform(200, 2800, (80, 2))
    truth = np.array([.6, .04, .02, .025, -.65, -.05, .035, -.07, .045])
    pixels = project((world-WORLD_CENTRE)/WORLD_SCALE, truth)*PIXEL_SCALE+PIXEL_CENTRE
    fitted = fit(world, pixels, 1)
    assert np.max(np.abs(invert(pixels, fitted)-world)) < .001
    assert abs(fitted[8]-truth[8]) < 1e-5
    assert np.max(np.abs(invert(pixels, truth)-world)) < .001
    truth2 = np.append(truth, .012)
    pixels2 = project((world-WORLD_CENTRE)/WORLD_SCALE, truth2)*PIXEL_SCALE+PIXEL_CENTRE
    fitted2 = fit(world, pixels2, 2)
    assert np.max(np.abs(invert(pixels2, fitted2)-world)) < .001
    assert np.max(np.abs(fitted2[8:]-truth2[8:])) < 1e-5


def ideal_pixels(pixels, parameters):
    centre = parameters[10:12] if len(parameters) == 12 else np.zeros(2)
    distorted = (pixels-PIXEL_CENTRE)/PIXEL_SCALE-centre
    rd = np.linalg.norm(distorted, axis=1)
    ru = rd.copy()
    k1, k2 = parameters[8:10]
    for _ in range(15):
        r2 = ru**2
        update = (ru*(1+k1*r2+k2*r2**2)-rd)/(1+3*k1*r2+5*k2*r2**2)
        ru -= update
        if np.max(np.abs(update)) < 1e-11:
            break
    ratio = np.divide(ru, rd, out=np.ones_like(ru), where=rd > 0)
    return distorted*ratio[:, None]+centre


def model_project(world_mm, parameters):
    q = homography((world_mm-WORLD_CENTRE)/WORLD_SCALE, matrix_of(parameters))
    centre = parameters[10:12] if len(parameters) == 12 else np.zeros(2)
    q -= centre
    r2 = np.sum(q**2, axis=1)
    q *= (1+parameters[8]*r2+parameters[9]*r2**2)[:, None]
    return (q+centre)*PIXEL_SCALE+PIXEL_CENTRE


def model_invert(pixels, parameters):
    return homography(ideal_pixels(pixels, parameters), np.linalg.inv(matrix_of(parameters)))*WORLD_SCALE+WORLD_CENTRE


def valid_model(parameters):
    if not np.all(np.isfinite(parameters)) or (len(parameters) == 12 and np.max(np.abs(parameters[10:])) > .15):
        return False
    r2 = np.linspace(0, 2, 100)
    if np.min(1+parameters[8]*r2+parameters[9]*r2**2) < .5:
        return False
    if np.min(1+3*parameters[8]*r2+5*parameters[9]*r2**2) < .2:
        return False
    return True


def refresh_pose(world, pixels, lens):
    """Fit exactly eight pose parameters, keeping the radial model fixed."""
    h, _ = cv2.findHomography((world-1500)/1500, ideal_pixels(pixels, lens), 0)
    pose = (h/h[2, 2]).ravel()[:8]
    fixed = np.asarray(lens)[8:]
    def residual(p):
        return (model_project(world, np.r_[p, fixed])-pixels).ravel()
    damping, r = .001, residual(pose)
    for _ in range(60):
        jacobian = np.column_stack([(residual(pose+np.eye(8)[j]*1e-6)-r)/1e-6 for j in range(8)])
        normal = jacobian.T@jacobian
        step = np.linalg.solve(normal+damping*np.diag(np.maximum(np.diag(normal), 1e-8)), -jacobian.T@r)
        proposal = pose+step
        rr = residual(proposal)
        if valid_model(np.r_[proposal, fixed]) and rr@rr < r@r:
            improvement = r@r-rr@rr
            pose, r = proposal, rr
            damping = max(1e-10, damping*.3)
            if improvement < 1e-8 or np.max(abs(step)) < 1e-9:
                break
        else:
            damping *= 10
            if damping > 1e12:
                break
    return np.r_[pose, fixed], float(np.sqrt(r@r/len(world)))


def expand_corners(points, inset_pixels):
    """Move each edge outward by a constant pixel amount, not a metric scaling."""
    points = np.asarray(points)
    centre = points.mean(axis=0)
    mids = (points+np.roll(points, -1, axis=0))/2
    edges = np.roll(points, -1, axis=0)-points
    normals = np.column_stack([edges[:, 1], -edges[:, 0]])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    normals *= np.where(np.sum(normals*(mids-centre), axis=1) > 0, 1, -1)[:, None]
    moves = np.array([np.linalg.solve(np.array([normals[(i-1)%4], normals[i]]), [1., 1.]) for i in range(4)])
    return points+inset_pixels*moves

