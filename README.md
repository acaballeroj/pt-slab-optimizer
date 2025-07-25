# 🏗️ Post-Tensioned Slab Optimizer – Python Prototype

This repository contains a simplified Python implementation of a structural optimization method for post-tensioned (PT) slabs, inspired by the academic work of Farid et al. (2024).  
It accompanies the [LinkedIn article](https://www.linkedin.com/pulse/automating-design-post-tensioned-slabs-practical-implementation) on **automating practical PT slab design** workflows.

---

## 🔍 Objective
Demonstrate how an *influence matrix-based approach* can be used to:
- Optimize the number of strands per tendon
- Avoid iterative structural reanalysis
- Enable smarter early-stage or value engineering decisions

---

## 📐 Key Concepts
- Structural influence matrix  
- Optimization using `cvxpy`  
- Stress constraints at control points  
- Lightweight integration logic with tools like SAFE, SOFiSTiK, PLAXIS or BIM platforms

---

## 💻 Implementation Details
This is a **simplified prototype**, not a commercial design tool.

- Language: `Python 3`
- Libraries used:
  - `cvxpy` for constrained optimization
  - `numpy` and `matplotlib` for logic and visualization

The implementation includes:
- Influence matrix generation
- Load-induced stress input
- Optimization to minimize PT steel
- Example case: 8×12m slab, simple boundary conditions

---

## 🚀 How to Use
1. Clone or download the repository  
   ```bash
   git clone https://github.com/YOUR_USERNAME/pt-slab-optimizer.git
   ```
2. Install dependencies  
   ```bash
   pip install -r requirements.txt
   ```
3. Run the main file  
   ```bash
   python pt_slab_optimizer.py
   ```
4. Review results, charts, and stress distribution

---

## 🧪 Example Output
The algorithm shows a ~30% reduction in PT steel for a basic test slab compared to uniform strand layouts.

---

## ⚠️ Disclaimer
This repository is intended for **educational and exploratory purposes** only.  
It does **not** replace full structural design workflows or code-based verification.

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).

---

## 🤝 Contributions
Feel free to fork the project, report issues, or propose improvements.  
If you're working on **digitalization of structural engineering**, I’d be happy to connect.
