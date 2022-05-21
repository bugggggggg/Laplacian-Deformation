// 整个衣服ppsition， 变形部分index
class Tool {
    static init_two_demension_array(x, y) {
        console.assert(x >= 0 && y >= 0, "数组大小必须为非负")
        var arr = new Array(x)
        for (var i = 0; i < x; i++) {
            arr[i] = new Array(y).fill(0)
        }
        return arr
    }

    static get_mat_data2(mat) { // 2维
        // console.log(mat)
        var size = mat.size()
        console.assert(size.length <= 2, `${mat}矩阵最多为2维`)
        if (size.length == 2)
            return math.subset(mat, math.index(math.range(0, size[0]), math.range(0, size[1])))
        return math.subset(mat, math.index(math.range(0, size[0])))
    }

    static unit_vector (arr) {
        var norm = 0
        arr.forEach(function (w, index) {
            norm += w * w
        })
        norm = math.sqrt(norm)
        var new_arr = arr.slice(0)
        new_arr.forEach(function (w, index) {
            new_arr[index] /= norm
        })
        return new_arr
    }

    static submesh (points, faces, constrain) {
        var new_points = []
        var new_faces = []
        var points_mapping = [] // submesh的点与原mesh点的映射关系
        var points_map = new Map() // points_mapping的逆
        points.forEach(function (point, index) {
            if (constrain(point)) {
                points_map.set(index, points_mapping.length)
                points_mapping.push(index)
                new_points.push(point)
            }
        })
        faces.forEach(function (face, index) {
            var new_face = []
            face.forEach(function (vertex, _) {
                if (constrain(points[vertex]))
                    new_face.push(points_map.get(vertex))
            })
            if (new_face.length === 3)
                new_faces.push(new_face)
        })
        return {points: new_points, faces: new_faces, mapping: points_mapping}
    }

    static cal_zero_ratio_in_mat2 (mat_data) {
        var zero = 0
        var all = 0
        for (var i = 0; i < mat_data.length; i++) {
            for (var j = 0; j < mat_data[i].length; j++) {
                all++;
                if (math.abs(mat_data[i][j]) < 1e-7)
                    zero++;
            }
        }
        console.log(mat_data)
        console.log(zero, all)
        return zero / all
    }

    static arr_add(arr, w) {
        var new_arr = arr.slice(0)
        new_arr.forEach(function (_, index) {
            new_arr[index] += w
        })
        return new_arr
    }

    static arr_sub(arr, w) {
        var new_arr = arr.slice(0)
        new_arr.forEach(function (_, index) {
            new_arr[index] -= w
        })
        return new_arr
    }
}


class Rotation_Tool {
    static rotate(points, axis, angle, origin = null) {
        if (origin == null) origin = [0, 0, 0]
        console.assert(origin.length == 3, "原点必须为3维")

        angle = angle / 180 * Math.PI
        axis = Tool.unit_vector(axis)
        // rotate_mat = e ^ (I * axis * angle)
        var rotate_mat = new Array()
        for (var i = 0; i < 3; i++) rotate_mat.push(new Array(3).fill(0))
        for (var i = 0; i < 3; i++) {
            var arr = new Array(3).fill(0)
            arr[i] = 1
            arr = math.cross(arr, axis)
            for (var j = 0; j < 3; j++) {
                rotate_mat[j][i] = angle * arr[j]
            }
        }
        // console.log(`rotate_mat: `, rotate_mat)
        // console.log(`rotate_mat: `, math.expm(math.matrix(rotate_mat)))
        rotate_mat = Tool.get_mat_data2(math.expm(math.matrix(rotate_mat)))

        console.log(rotate_mat)
        var new_points = []
        for (var i = 0; i < points.length; i++) {
            var point = points[i].slice(0)
            point.forEach(function (_, index) {
                point[index] -= origin[index]
            })
            // console.log("point: ", point)
            var new_point = math.multiply(rotate_mat, math.transpose(math.matrix(point)))._data
            // console.log("new_point: ", new_point)
            new_point.forEach(function (_, index) {
                new_point[index] += origin[index]
            })
            // console.log(point)
            new_points.push(new_point)
        }

        return new_points
    }
}


class Anchor_Selector {
    ratio = 0.1
    constructor(constrain, points, linking_edges = null) {
        // console.log(points)
        this.points = new Array()
        this.points_map = new Array() //满足条件的点和原模型点的对应关系
        for (var i = 0; i < points.length; i++) {
            if (constrain == null || constrain(points[i])) {
                this.points.push(points[i])
                this.points_map.push(i)
            }
        }

        this.num_points = this.points.length
        this.linking_edges = linking_edges // 只随机选点，用不到边
    }

    select_random() {
        var permutation = new Array()
        for (var i = 0; i < this.points.length; i++) {
            permutation.push(i)
        }
        var select_indexes = math.pickRandom(permutation, math.max(1, this.ratio * this.num_points)) 

        for (var i = 0; i < select_indexes.length; i++) {
            select_indexes[i] = this.points_map[select_indexes[i]]
        }
        return select_indexes
    }

    select_all() {
        var permutation = []
        for (var i = 0; i < this.points.length; i++) {
            permutation.push(i)
        }
        var select_indexes = []

        for (var i = 0; i < permutation.length; i++) {
            select_indexes.push(this.points_map[permutation[i]])
        }
        return select_indexes
    }

    select_bfs() {
        let select_indexes = [0]
        let dep = []
        for (let i = 0; i < this.points.length; i++) {
            dep.push(1e9)
        }
        while (select_indexes.length < this.ratio * this.points.length) {
            let origin = select_indexes[select_indexes.length - 1]
            let queue = [origin]
            dep[origin] = 0
            let head = 0
            while (head < queue.length) {
                let x = queue[head]
                head++;
                for (let i = 0; i < this.linking_edges[x].length; i++) {
                    let y = this.linking_edges[x][i]
                    if (dep[y] > dep[x] + 1) {
                        dep[y] = dep[x] + 1
                        queue.push(y)
                    }
                }
            }
            let select = 0
            for (let i = 0; i < dep.length; i++) {
                if (dep[i] > dep[select]) {
                    select = i
                }
            }
            select_indexes.push(select)
        }
        return select_indexes
    }
}


class Laplacaian_Deformation {
    constructor(points, faces, constrain = null) {
        this.weight = 0.5 // 形变的全重
        this.points = points
        this.num_points = this.points.length

        this.linking_edges = this.get_linking_edges(faces)
        this.laplacian_mat_data = this.get_laplacian_mat_data_from_array()

        this.laplacian_position_mat_data = math.multiply(this.laplacian_mat_data, this.points)

        // this.anchor_indexes = new Anchor_Selector(constrain, this.points).select_all()
        this.anchor_indexes = new Anchor_Selector(constrain, this.points, this.linking_edges).select_bfs()
        this.revise_laplacian_mat_data_by_anchor()

        // this.laplacian_coefficient_data = this.pre_calculate_laplacian_coefficient_data()
        // console.log(this.laplacian_coefficient_data)
        this.laplacian_coefficient_data = this.pre_calculate_laplacian_coefficient_data_quickly()
        console.log("init complete")
    }

    init_submesh_mapping_and_global_positions(mapping, positions) {
        this.submesh_mapping = mapping
        this.global_positions = positions
    }

    get_deformed_global_positions(deformed_points) {
        console.assert(this.submesh_mapping != null, "未初始化submesh_mapping")
        console.assert(this.global_positions != null, "未初始化模型全局坐标")

        var new_global_positions = this.global_positions.slice(0)
        for (var i = 0; i < deformed_points.length; i++) {
            var id = this.submesh_mapping[i]
            for (var j = 0; j < 3; j++) {
                new_global_positions[j + 3 * id] = deformed_points[i][j]
            }
        }
        return new_global_positions
    }

    get_laplacian_mat() {
        return math.matrix(this.laplacian_mat_data)
    }

    get_laplacian_position_mat() {
        return math.matrix(this.laplacian_position_mat_data)
    }

    // entry
    deformation(axis, angle, origin) {
        var new_points = Rotation_Tool.rotate(this.points, axis, angle, origin)
        // console.log("rotated_select_points", rotated_select_points)

        // console.log("laplacian_position_mat: ", this.laplacian_position_mat_data)
        return this.deformation_with_new_points(new_points)
    }

    //entry
    deformation_with_new_points(new_points_array) {
        var new_points = []
        for (var i = 0; i < this.submesh_mapping.length; i++) {
            var pos = []
            var id = this.submesh_mapping[i]
            for (var j = id * 3; j < id * 3 + 3; j++) {
                pos.push(new_points_array[j])
            }
            new_points.push(pos)
        }

        var rotated_select_points = []
        for (var i = 0; i < this.anchor_indexes.length; i++)
            rotated_select_points.push(new_points[this.anchor_indexes[i]])

        // var new_laplacian_position = this.revise_laplacian_position_mat_data(new_points)

        for (var i = 0; i < rotated_select_points.length; i++) {
            var point = rotated_select_points[i]
            var new_point = []
            for (var j = 0; j < 3; j++) {
                new_point.push(point[j] * this.weight)
            }
            // new_laplacian_position.push(new_point)
            this.laplacian_position_mat_data.push(new_point)
        }
        // var ret_points = math.multiply(this.laplacian_coefficient_data, new_laplacian_position)
        var ret_points = math.multiply(this.laplacian_coefficient_data, this.laplacian_position_mat_data)

        for (var i = 0; i < this.anchor_indexes.length; i++) { //还原数据
            this.laplacian_position_mat_data.pop()
        }

        return this.get_deformed_global_positions(ret_points._data)
    }

    revise_laplacian_position_mat_data(new_points) {
        var new_laplacian_position = []
        for (var i = 0; i < this.laplacian_position_mat_data.length; i++) {
            var coe = this.get_revise_coefficient(new_points, i)._data
            var s = coe[0], h1 = coe[1], h2 = coe[2], h3 = coe[3]
            var pos = this.laplacian_position_mat_data[i]
            // if (i === 0) {
            //     console.log("coe: ", coe)
            //     console.log("pos: ", pos)
            // }
            var new_pos = [
                s * pos[0] - h3 * pos[1] + h2 * pos[2],
                h3 * pos[0] + s * pos[1] - h1 * pos[2],
                -h2 * pos[0] + h1 * pos[1] + s * pos[2]]
            new_laplacian_position.push(new_pos)
        }
        return new_laplacian_position
    }

    get_revise_coefficient(new_points, i) {
        var pos = this.points[i]
        var A_data = [[pos[0], 0, pos[2], -pos[1]],
                      [pos[1], -pos[2], 0, pos[0]],
                      [pos[2], pos[1], -pos[0], 0]]
        var b_data = [new_points[i][0], new_points[i][1], new_points[i][2]]
        for (var j = 0; j < this.linking_edges[i].length; j++) {
            var y = this.linking_edges[i][j]
            pos = this.points[y]
            A_data.push([pos[0], 0, pos[2], -pos[1]])
            A_data.push([pos[1], -pos[2], 0, pos[0]])
            A_data.push([pos[2], pos[1], -pos[0], 0])
            b_data.push(new_points[y][0])
            b_data.push(new_points[y][1])
            b_data.push(new_points[y][2])
        }
        var A = math.matrix(A_data)
        var A_T = math.transpose(A)
        var tmp = math.multiply(A_T, A)
        tmp = math.inv(tmp)
        var ttmp = math.multiply(tmp, A_T)
        return Tool.get_mat_data2(math.multiply(ttmp, math.matrix(b_data)))
    }

    get_linking_edges(faces) {
        var linking_edges = Tool.init_two_demension_array(this.num_points, 0)
        faces.forEach(function(face, _) {
            console.assert(face.length === 3, `${face}必须为三角面`)
            for (var i = 0; i < 3; i++) {
                for (var j = i + 1; j < 3; j++) {
                    var x = math.min(face[i], face[j])
                    var y = math.max(face[i], face[j])
                    console.assert(linking_edges[x] !== undefined, `${x}, ${y}`)
                    linking_edges[x].push(y)
                    linking_edges[y].push(x)
                }
            }
        })
        for (var i = 0; i < this.num_points; i++) {
            // console.log(linking_edges[i])
            // console.log([... new Set(linking_edges[i])])
            linking_edges[i] = [... new Set(linking_edges[i])]
        }
        return linking_edges
    }

    get_laplacian_mat_data_from_array() { 
        var laplacian_mat_arr = Tool.init_two_demension_array(this.num_points, this.num_points)
        for (var i = 0; i < this.num_points; i++) {
            var deg = this.linking_edges[i].length
            // console.assert(deg > 0, `${i}:度数要作为除数,必须大于0`)
            laplacian_mat_arr[i][i] = 1
            this.linking_edges[i].forEach(function (vertex, _) {
                // console.log(i, vertex)
                laplacian_mat_arr[i][vertex] = - 1.0 / deg 
            })
        }
        // console.log("laplacian_mat_arr: ", laplacian_mat_arr)
        return laplacian_mat_arr
    }// return math.matrix(laplacian_mat_arr, 'sparse') 是反着的,[[1], [2]]得到2*1矩阵,且用math.mutiply很奇怪

    revise_laplacian_mat_data_by_anchor() {  //把选好的锚点index加上
        for (var i = 0; i < this.anchor_indexes.length; i++) {
            var index = this.anchor_indexes[i]
            var arr = new Array(this.num_points).fill(0)
            arr[index] = this.weight
            this.laplacian_mat_data.push(arr)
        }
    }

    pre_calculate_laplacian_coefficient_data() { //提前计算(L^T * L)^(-1) * L^T
        var L = this.get_laplacian_mat()
        // console.log(L)
        var L_T = math.transpose(L)
        
        // console.log(L_T)
        var tmp = math.multiply(L_T, L)
        tmp = math.inv(tmp)
        let ttmp = math.multiply(tmp, L_T)
        console.log(ttmp.size())
        return Tool.get_mat_data2(ttmp)
    }

    pre_calculate_laplacian_coefficient_data_quickly() {
        let height = this.num_points + this.anchor_indexes.length
        let width = this.num_points
        let triplet = new Triplet(height, width)
        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {
                if (this.laplacian_mat_data[i][j] !== 0) {
                    triplet.addEntry(this.laplacian_mat_data[i][j], i, j)
                }
            }
        }

        let L = SparseMatrix.fromTriplet(triplet)
        let L_T = L.transpose()
        let tmp = L_T.timesSparse(L)
        let ttmp = this.cal_matrix_inv(tmp)

        let ans = ttmp.timesSparse(L_T).toDense()
        let ret_data = []
        for (let i = 0; i < width; i++) {
            let row_data = []
            for (let j = 0; j < height; j++) {
                row_data.push(ans.get(i, j))
            }
            ret_data.push(row_data)
        }
        return Tool.get_mat_data2(math.matrix(ret_data))
        // console.log(`${width}, ${height}`)
        // return ret_data
    }

    cal_matrix_inv(L) {
        let chol = L.chol()
        let I = DenseMatrix.identity(L.nRows(), L.nRows())
        let L_inv = chol.solvePositiveDefinite(I)
        let L_inv_triplet = new Triplet(L.nRows(), L.nCols())
        for (let i = 0; i < L.nRows(); i++) {
            for (let j = 0; j < L.nCols(); j++) {
                L_inv_triplet.addEntry(L_inv.get(i, j), i, j)
                // console.log(`${i}, ${j}, ${L_inv.get(i, j)}`)
            }
        }
        return SparseMatrix.fromTriplet(L_inv_triplet)
    }

}



function test() {
    let data = new Triplet(3, 3)
    data.addEntry(1, 0, 0)
    data.addEntry(4, 0, 1)
    data.addEntry(6, 0, 2)
    data.addEntry(2, 1, 1)
    data.addEntry(3, 2, 2)
    let L = SparseMatrix.fromTriplet(data)

}


