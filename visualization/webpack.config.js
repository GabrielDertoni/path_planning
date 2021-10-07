const path = require("path");
const CopyPlugin = require("copy-webpack-plugin");
const WasmPackPlugin = require("@wasm-tool/wasm-pack-plugin");
const HtmlWebpackPlugin = require("html-webpack-plugin");

const dist = path.resolve(__dirname, "dist");

module.exports = {
  mode: "development",
  entry: path.resolve(__dirname, "./ts/index.tsx"),
  // entry: "./bootstrap.js",
  resolve: { 
    extensions: [".ts", ".tsx", ".js", ".wasm"],
  },
  output: {
    path: path.resolve(__dirname, "./dist"),
    filename: "index.js"
  },
  devServer: {
    static: {
      directory: path.resolve(__dirname, "static"),
      staticOptions: {},
      watch: true,
    }
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: "./static/index.html"
    }),

    new WasmPackPlugin({
      crateDirectory: __dirname,
      withTypeScript: true,
    }),
  ],
  module: {
    rules: [{
      test: /\.(js|ts)x?$/,
      exclude: /node_modules/,
      loader: "babel-loader"
    }]
  },
  experiments: {
    asyncWebAssembly: true
  }
};
