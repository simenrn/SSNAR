/**
 * This code is written as part of a Master Thesis
 * the spring of 2017.
 *
 * Lars Marius Strande (Master 2017 @ NTNU)
 */
package no.ntnu.et.mapping;

import java.util.HashMap;
import no.ntnu.et.general.Position;
import org.ejml.simple.SimpleMatrix;
import org.ejml.simple.SimpleSVD;

/**
 *
 * @author larsmast
 */
public class TransformationAlg {

    //sett one
    SimpleMatrix S_ref;

    //sett two
    SimpleMatrix S_new;

    // rotation matrix
    SimpleMatrix rotation;

    // translation matrix
    SimpleMatrix translation;

    public TransformationAlg(SimpleMatrix S_ref, SimpleMatrix S_new) throws IllegalArgumentException {
        if (!isValidInputMatrix(S_ref) || !isValidInputMatrix(S_new)) {
            throw new IllegalArgumentException();
        }
        if (S_ref.numCols() != S_new.numCols() || S_ref.numRows() != S_new.numRows()) {
            throw new IllegalArgumentException();
        }
        this.S_ref = S_ref;
        this.S_new = S_new;
        calculate();
    }

    /**
     * Check if the given matrix is a valid matrix for the algorithm.
     *
     * @param M The matrix to be checked.
     * @return True if and only if the number of columns is at least two and the
     * number of rows is at least the number of columns.
     */
    private boolean isValidInputMatrix(SimpleMatrix M) {
        return M.numCols() >= 2 && M.numRows() >= M.numCols();
    }

    private SimpleMatrix getCovariance() {
        HashMap<String, SimpleMatrix> centroids = getCenterOfMass();
        SimpleMatrix S_refCentroid = centroids.get("S_ref");
        SimpleMatrix S_newCentroid = centroids.get("S_new");
        int n = S_ref.numCols();
        int m = S_ref.numRows();
        SimpleMatrix A = new SimpleMatrix(new double[n][n]);
        for (int j = 0; j < m; j++) {
            SimpleMatrix S_refJ = new SimpleMatrix(1, n);
            SimpleMatrix S_newJ = new SimpleMatrix(1, n);
            for (int i = 0; i < n; i++) {
                S_newJ.set(0, i, S_new.get(j, i) - S_newCentroid.get(0, i));
                S_refJ.set(0, i, S_ref.get(j, i) - S_refCentroid.get(0, i));
            }
            SimpleMatrix S = S_newJ.transpose().mult(S_refJ);
            A = A.plus(S);
        }
        return A;
    }

    /**
     * Compute the Singular Value Decomposition of the Covariance Matrix. Use
     * the results to create the rotation and translation matrix.
     */
    private void calculate() {
        // finding svd
        SimpleMatrix A = getCovariance();
        SimpleSVD<SimpleMatrix> svd = new SimpleSVD<SimpleMatrix>(A.getMatrix(), true);
        SimpleMatrix V = svd.getU().transpose();
        SimpleMatrix W = svd.getV();

        // Corrections
        SimpleMatrix R = W.mult(V);
        int d = (int) Math.signum(R.determinant());
        System.out.println("D = " + d);
        // optimal rotation matrix
        SimpleMatrix S = SimpleMatrix.identity(S_ref.numCols());
        S.set(S_ref.numCols() - 1, S_ref.numCols() - 1, d);
        this.rotation = W.mult(S).mult(V);

        // trnslation matrix
        HashMap<String, SimpleMatrix> centroids = getCenterOfMass();
        SimpleMatrix S_refCentroid = centroids.get("S_ref");
        SimpleMatrix S_newCentroid = centroids.get("S_new");
        this.translation = R.mult(S_refCentroid.transpose()).scale(-1).plus(S_newCentroid.transpose());
    }

    /**
     * Compute the centroids for matrix S_ref and S_new
     *
     * @return HashMap containing the center of mass for matrix S_ref and S_new
     */
    private HashMap<String, SimpleMatrix> getCenterOfMass() {
        HashMap<String, SimpleMatrix> result = new HashMap<String, SimpleMatrix>();
        int n = S_ref.numCols();
        int m = S_ref.numRows();
        SimpleMatrix S_refCentroid = new SimpleMatrix(1, n);
        SimpleMatrix S_newCentroid = new SimpleMatrix(1, n);
        for (int i = 0; i < n; i++) {
            double sumS_ref = 0;
            double sumS_new = 0;
            for (int j = 0; j < m; j++) {
                sumS_ref += S_ref.get(j, i);
                sumS_new += S_new.get(j, i);
            }
            S_refCentroid.set(0, i, sumS_ref / m);
            S_newCentroid.set(0, i, sumS_new / m);
        }
        result.put("S_ref", S_refCentroid);
        result.put("S_new", S_newCentroid);
        return result;
    }

    /**
     * Get the translation vector.
     *
     * @return The translation vector.
     */
    public SimpleMatrix getTranslationVek() {
        return this.translation;
    }

    /**
     * Get the translation matrix.
     *
     * @return The translation matrix.
     */
    public SimpleMatrix getTranslationMatrix() {
        double[][] s = new double[3][3];
        SimpleMatrix transVec = getTranslationVek();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                s[j][i] = transVec.get(j, 0);
            }
        }
        return new SimpleMatrix(s);
    }

    public SimpleMatrix homogeneousTransformation(SimpleMatrix r, SimpleMatrix t) {
        SimpleMatrix m = SimpleMatrix.identity(3);
        m.combine(0, 0, r);
        m.combine(3, 3, r);

        return m;
    }

    /**
     * Get the rotation matrix
     *
     * @return The rotation matrix
     */
    public SimpleMatrix getRotation() {
        return this.rotation;
    }
}
