from init_corresponding_pts import corresponding_pts_init

import os
import cv2
import numpy as np
from scipy import optimize

"""
1. Get initial corresponding points manually
2. Normalize coordinates, estimate Fundamental matrix using Linear LS
3. Refine F using Non-Linear least squares and triangulation
4. Rectify images
5. Find corresponding points in rectified image
6. Use RANSAC to get rid of outliers
7. Estimate F using Linear least squares
8. Refine F using Non-Linear least squares and triangulation
9. 3D projection using triangulation of initial points chosen manually in each image
"""


def write_to_latex(results_dir, mat, var_name):
    """
    Writes a matrix in the form var_name = mat
    :param results_dir: The directory where file named "Latex" would be written
    :param mat: Matrix to write to Latex
    :param var_name: Name of variable to write to Latex. Make sure it follows Latex format(For ex. if you want the var name to have "_")
    :return:
    """

    with open(os.path.join(results_dir, "Latex"), 'a+') as fh:
        st = " \\\\".join([" & ".join(map('{0:.6f}'.format, line)) for line in mat])
        st = "\\begin{bmatrix}" + st + "\end{bmatrix}"
        fh.write("{} = {}\n".format(var_name, st))

        fh.write("\\newline \n")


class ProjectiveReconstruction:
    def __init__(self, img_1_path, img_2_path, results_dir):
        """

        :param img_1_path:
        :param img_2_path:
        :param results_dir:
        """

        self.corr_pts_init_1 = corresponding_pts_init[img_1_path]  # List or no array of N x 2 (rows have (x,y) coordinates)
        self.corr_pts_init_2 = corresponding_pts_init[img_2_path]

        if isinstance(self.corr_pts_init_1, list):
            self.corr_pts_init_1 = np.array(self.corr_pts_init_1)
            self.corr_pts_init_1 = np.hstack(
                (self.corr_pts_init_1, np.ones((self.corr_pts_init_1.shape[0], 1))))  # Convert to homogenous (rows of x, y, 1)  shape = N x 3


        if isinstance(self.corr_pts_init_2, list):
            self.corr_pts_init_2 = np.array(self.corr_pts_init_2)
            self.corr_pts_init_2 = np.hstack(
                (self.corr_pts_init_2,
                 np.ones((self.corr_pts_init_2.shape[0], 1))))  # Convert to homogenous (rows of x, y, 1)  shape = N x 3

        self.img_1_path = img_1_path
        self.img_2_path = img_2_path

        if not os.path.exists(results_dir):
            os.makedirs(results_dir)

        self.results_dir = results_dir

    def _plot_pt_crd(self, img, crd, out_path=None):

        if isinstance(img, str):
            img = cv2.imread(img, 1)

        for i, xy in enumerate(crd):
            cv2.circle(img, (int(xy[0]), int(xy[1])), 10, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.putText(img, "{}".format(i + 1), (int(xy[0] - 10), int(xy[1]) - 10), 0, 3.0, (0, 255, 0), 4)

        if out_path:
            cv2.imwrite(out_path, img)

    def _build_F_homogenous_eqns(self, img_1_crd_hc, img_2_crd_hc):
        """

        :param img_1_crd_hc: HC representation shape: N x 3 (rows of x, y, 1)
        :param img_2_crd_hc: HC representation shape: N x 3 (rows of x', y', 1)
        :return: A : shape: N x 9
        """
        # Equation : [x'x, x'y, x', y'x, y'y, y', x, y, 1]

        A = np.hstack((img_1_crd_hc * img_2_crd_hc[:, 0:1], img_1_crd_hc * img_2_crd_hc[:, 1:2], img_1_crd_hc))

        return A

    def calculate_F_Linear_LS(self, img_1_crd_hc, img_2_crd_hc):
        """
        :param img_1_crd: N x 2 coordinates (rows of x, y)
        :param img_2_crd: N x 2 coordinates (rows of x, y)
        :return:
        """

        def _normalize_crd(img_crd_hc):
            """
            :param img_crd: shape N x 2 (rows of (x, y))
            :return:
            """

            mu = np.mean(img_crd_hc, axis=0)
            var = np.sum(np.var(img_crd_hc, axis=0))  # var = var_x + var_y

            scale = np.sqrt(2 / var)

            T = np.zeros((3, 3))

            T[0][0] = T[1][1] = scale
            T[2][2] = 1

            T[0:2, -1] = - mu[0:2] * scale

            norm_crd_hc = np.dot(T, img_crd_hc.T)  # shape = 3 x N

            norm_crd_hc = norm_crd_hc / norm_crd_hc[-1, :]

            return T, norm_crd_hc.T

        # Normalize
        T_1, norm_img_1_crd_hc = _normalize_crd(img_1_crd_hc)
        T_2, norm_img_2_crd_hc = _normalize_crd(img_2_crd_hc)

        assert norm_img_1_crd_hc.shape == img_1_crd_hc.shape, "Shape of normalized and non should be same \n"
        assert norm_img_2_crd_hc.shape == img_2_crd_hc.shape, "Shape of normalized and non should be same \n"

        # Build equations
        A = self._build_F_homogenous_eqns(norm_img_1_crd_hc, norm_img_2_crd_hc)

        # Compute LS solution : Linear Least squares solution: eigen vec of A.T*A with min eigen val
        U, sig, Vh = np.linalg.svd(A)
        F = Vh[-1, :].reshape((3, 3))

        # Enforce Rank 2 constraint by setting min eigen val to 0
        U, sig, Vh = np.linalg.svd(F)
        sig[-1] = 0
        F = np.dot(U, np.dot(np.diag(sig), Vh))

        # De-normalize F
        F = np.matmul(T_2.T, np.matmul(F, T_1))
        F = F / F[2][2]  # Since F is homogenous

        # te = np.dot(img_crd_hc_2, np.dot(F, img_crd_hc_1.T))  # x_2.T*F*x_1
        # te = np.diag(te)
        #
        # print(te)

        write_to_latex(self.results_dir, F, "F_{init-8pt}")

        return F

    @staticmethod
    def compute_residuals_F_Non_Linear_LS(F_init_vec, corr_pts_hc_1, corr_pts_hc_2):

        F_init_mat = F_init_vec.reshape((3, 3))

        P_1, P_2 = ProjectiveReconstruction._build_initial_canonical_camera_matrices(F_init_mat)

        X = ProjectiveReconstruction.triangulate(P_1, P_2, corr_pts_hc_1, corr_pts_hc_2)

        x_proj_1 = np.dot(P_1, X.T)  # shape: 3 x N
        x_proj_1 = x_proj_1/x_proj_1[-1, :]
        x_proj_1 = x_proj_1.T  #  out shape: N x 3

        x_proj_2 = np.dot(P_2, X.T)  # shape: 3 x N
        x_proj_2 = x_proj_2 / x_proj_2[-1, :]
        x_proj_2 = x_proj_2.T  # out shape: N x 3
        residual = np.hstack((np.linalg.norm((corr_pts_hc_1 - x_proj_1)**2, axis=1), np.linalg.norm((corr_pts_hc_2 - x_proj_2)**2, axis=1)))
          # Since cost is 0.5 * sum(rho(f_i(x)**2) where rho is fun f(z) = z

        return residual

    def calculate_F_Non_Linear_LS(self, F_init, corr_pts_hc_1, corr_pts_hc_2):

        sol = optimize.least_squares(ProjectiveReconstruction.compute_residuals_F_Non_Linear_LS, F_init.ravel(), args=(corr_pts_hc_1, corr_pts_hc_2),
                                     method='lm',
                                     xtol=1e-15, ftol=1e-15)

        F_LM = np.reshape(sol.x, (3,3))

        F_LM = F_LM/F_LM[2][2]

        return F_LM


    @staticmethod
    def _build_initial_canonical_camera_matrices(F):
        """
        Function to build canonical camera projection matrices P_1, P_2 from F
        P_1 = [I|0]
        P_2 = [[E_2]*F|e_2]  e_2 -> 2nd camera epipole
        :param F: Fundamental matrix
        :return:
        """

        P_1 = np.hstack((np.eye(3), np.zeros((3, 1))))  # P1 = [I3|0] shape = 3 x 4

        U, sig, Vh = np.linalg.svd(F)

        # Left null space is the col of U corresponding to zero singular value.

        id = np.argmin(sig)  # this is redundant as sig is arranged in decreasing order

        e_2 = U[:, id:id + 1]  # 2nd camera epipole

        e_2_mat = ProjectiveReconstruction.convert_vec_to_mat_cross_prod(e_2)

        P_2 = np.hstack((np.dot(e_2_mat, F), e_2))  # P_2 =  [[E_2]*F|e_2]

        assert P_1.shape == P_2.shape, "Camera Matrices not of same shape \n"

        return P_1, P_2

    @staticmethod
    def convert_vec_to_mat_cross_prod(vec):
        """
        Function to convert a vector to matrix notation to be used when performing cross-product
        a x b = [A]*b  where a, b are vectors,
        [A] is the matrix representation of vector A,
        x indicates cross product
        * indicates matrix product

        :param vec: 3 dim vector
        :return:
        """

        mat = np.zeros((3, 3))

        mat[0][1] = -vec[2]
        mat[0][2] = vec[1]

        mat[1][0] = vec[2]
        mat[1][2] = -vec[0]

        mat[2][0] = -vec[1]
        mat[2][1] = vec[0]

        return mat

    @staticmethod
    def triangulate(P_1, P_2, corr_pts_hc_1, corr_pts_hc_2):
        """

        :param P_1: Camera projection matrices for first camera -> [P_11.T, P_12.T, P_13.T] rows
        :param P_2: Camera projection matrices for second camera -> [P_21.T, P_22.T, P_23.T] rows
        :param corr_pts_hc_1: Corresponding points homogenous crd in first image (shape: N x 3)
        :param corr_pts_hc_2: Corresponding points homogenous crd in second image (shape: N x 3)
        :return:
        """

        N = corr_pts_hc_1.shape[0]  # Num of points

        p3 = np.vstack(
            (P_1[2:3, :], P_1[2:3, :], P_2[2:3, :], P_2[2:3, :]))  # [P_13.T, P_13.T, P_23.T, P_23.T]: shape = 4 x 4

        p3 = np.repeat(p3[np.newaxis, :, :], N, axis=0)  # shape = N x 4 x 4 : repeat the 2D 4x4 matrix along the 3rd dimension

        p12 = np.vstack((P_1[0:1, :], P_1[1:2, :], P_2[0:1, :], P_2[1:2, :]))  # [P_11.T, P_12.T, P_21.T, P_22.T]: shape = 4 x 4

        p12 = np.repeat(p12[np.newaxis, :, :], N, axis=0)  # shape = N x 4 x 4 : repeat the 2D 4x4 matrix along the 3rd dimension

        mat_x = np.hstack((corr_pts_hc_1[:, 0:2], corr_pts_hc_2[:, 0:2]))  # shape N x 4: rows of x_1, y_1, x_2, y_2

        mat_x = mat_x.reshape(N, mat_x.shape[1], 1)  # shape: N x 4 x 1

        mat_A = mat_x * p3 - p12

        U, sig, Vh = np.linalg.svd(mat_A)

        X_hc = Vh[:, -1, :]  # Col corresponding to min eigen value of A.T*A : Shape = N x 4

        X_hc = X_hc/X_hc[:, 3:4]


        #### Testing ######

        # X = np.zeros((N, 4, 4))
        #
        #
        # for i in range(N):
        #     X[i][0][:] = corr_pts_hc_1[i][0] * P_1[2:3, :] - P_1[0:1, :]
        #     X[i][1][:] = corr_pts_hc_1[i][1] * P_1[2:3, :] - P_1[1:2, :]
        #
        #     X[i][2][:] = corr_pts_hc_2[i][0] * P_2[2:3, :] - P_2[0:1, :]
        #     X[i][3][:] = corr_pts_hc_2[i][1] * P_2[2:3, :] - P_2[1:2, :]
        #
        #     print("Checking if eqns same : {}".format(np.all(np.equal(mat_A[i], X[i]))))
        #
        #     U, sig, Vh = np.linalg.svd(X[i])
        #
        #     sol = Vh[-1, :]
        #
        #     sol = sol/sol[-1]
        #
        #     print("Checking if final solution is equal: {}".format(np.all(np.isclose(sol, X_hc[i]))))

        #### Testing ######

        return X_hc




    def compute_SIFT_features(self):
        pass

    def get_corresponding_pts(self):
        pass

    def rectify_image(self):
        pass

    def compute_canny_edges(self):
        pass

    def run(self):

        # Plot initial coordinates
        img_1 = cv2.imread(self.img_1_path)
        img_2 = cv2.imread(self.img_2_path)

        self._plot_pt_crd(img_1, self.corr_pts_init_1, os.path.join(self.results_dir, "init_pts_img_1.jpg"))
        self._plot_pt_crd(img_2, self.corr_pts_init_2, os.path.join(self.results_dir, "init_pts_img_2.jpg"))

        F_LS = self.calculate_F_Linear_LS(img_1_crd_hc=self.corr_pts_init_1, img_2_crd_hc=self.corr_pts_init_2)

        F_LM = self.calculate_F_Non_Linear_LS(F_init=F_LS, corr_pts_hc_1=self.corr_pts_init_1, corr_pts_hc_2=self.corr_pts_init_2)


        print("F after Linear Least Squares Normalized 8 point Algo:\n {}".format(F_LS))

        print("=========== ")
        print("F after Non Linear Least Squares Normalized 8 point Algo:\n {}".format(F_LM))


if __name__ == "__main__":
    img_1_path = "dataset/dataset_1/Pic_1.jpg"
    img_2_path = "dataset/dataset_1/Pic_2.jpg"

    results_dir = "dataset/dataset_1/results_12"

    obj = ProjectiveReconstruction(img_1_path, img_2_path, results_dir)
    obj.run()
