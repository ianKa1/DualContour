# Dual Contouring Artifact Diagnosis

---

## 🔥 1️⃣ Most Common Cause: QEF Solution Escapes the Cell (≈90% Probability)

If you did not clamp the QEF solution,

```
vertex ∉ cell bounds
```

the QEF may produce a solution that lies far away from the actual intersection points.

This typically happens when:

- The planes are nearly parallel  
- The normals are nearly collinear  
- The system has rank deficiency  

In these cases, the least-squares solution becomes unstable and can “explode.”

### ✅ Correct Fix

```cpp
if (!point_inside_cell(v))
    v = clamp_to_cell(v);
```

or:

```cpp
v = v.cwiseMax(cell_min).cwiseMin(cell_max);
```

This ensures the vertex remains inside the voxel cell.

---

## 🔥 2️⃣ QEF Degeneracy (Rank Deficiency)

If inside a cell:

- All normals are nearly collinear  
- Or there are only 1–2 Hermite samples  

Then matrix **A** has rank < 3.

In this case, the least-squares problem does not have a unique solution.

What happens:

- SVD may produce an extreme solution  
- The vertex may fly outside the cell  

### ✅ Fix

Check the smallest singular value:

```cpp
if (smallest_singular_value < epsilon)
    fallback_to_cell_center();
```

---

## 🔥 3️⃣ Inconsistent Normal Directions

If Hermite normals:

- Some point inward  
- Some point outward  

Then the plane constraints conflict.

The result:

- The vertex position oscillates wildly  
- The QEF becomes unstable  

### ✅ Check

Make sure:

```cpp
normal = normalize(gradient);
```

And ensure consistent orientation.

For example:

```
SDF > 0 → outside
```

All normals must follow the same convention.

---

## 🔥 4️⃣ Normals Not Normalized

QEF constructs equations of the form:

nᵢ · x = nᵢ · pᵢ

If nᵢ is not normalized:

- Some planes get excessive weight  
- The solution becomes biased  

Always normalize normals.

---

## 🔥 5️⃣ Incorrect Face Winding (Lower Probability)

If face indices are constructed incorrectly:

- Winding order may be wrong  
- Faces may flip  

However, this usually looks like topology errors rather than vertex explosions.

---

## 🔥 6️⃣ Grid Resolution Too Low

High-curvature regions (e.g., spout, handle) are sensitive.

If the grid is too coarse:

- Few Hermite samples per cell  
- QEF becomes unstable  

However, low resolution alone rarely causes extreme spikes.

---

## 🔬 Debug Test

After solving QEF, add:

```cpp
if (!point_inside_cell(v)) {
    std::cout << "QEF outside cell!" << std::endl;
}
```

You will likely see many warnings if this is the issue.

---

## 🧠 Why Dual Contouring Is Prone to This

QEF solves:

min ||Ax - b||²

This is an **unconstrained least squares problem**.

But Dual Contouring actually needs:

> A constrained least squares problem  
> (The solution must stay inside the cell.)

The original paper does not enforce this constraint explicitly.

So practical implementations must add clamping.

---

## 🚀 Recommended Robust Strategy

1. Solve the QEF  
2. If rank < 3 → fallback to cell center  
3. If solution is outside the cell → clamp  
4. If clamped solution still produces large error → fallback  

---

## 🎯 Final Summary

Artifacts are most likely caused by:

> QEF numerical instability + missing cell constraint

Fixing this should:

- Smooth the body  
- Stabilize the handle  
- Prevent exploding geometry  
