const path = require('path');
const webpack = require('webpack');

module.exports = {
  mode: 'development',
  entry: './src/components/ChatWidget.jsx',
  output: {
    path: path.resolve(__dirname, 'dist'),
    filename: 'widget.js',
    library: 'ChatbotWidget',
    libraryTarget: 'umd',
    globalObject: 'this',
  },
  module: {
    rules: [
      {
        test: /\.(js|jsx)$/,
        exclude: /node_modules/,
        use: {
          loader: 'babel-loader',
          options: {
            presets: ['@babel/preset-env', '@babel/preset-react']
          }
        }
      },
      {
        test: /\.css$/,
        use: [
          'style-loader',
          {
            loader: 'css-loader',
            options: {
              modules: false, // Don't use CSS modules for widget
            }
          }
        ]
      }
    ]
  },
  resolve: {
    extensions: ['.js', '.jsx']
  },
  devtool: 'eval-source-map',
  devServer: {
    static: {
      directory: path.join(__dirname, 'dist'),
    },
    compress: true,
    port: 3001, // Changed to avoid conflict
    hot: true,
    open: false, // Don't automatically open browser
    headers: {
      'Access-Control-Allow-Origin': '*',
      'Access-Control-Allow-Methods': 'GET, POST, PUT, DELETE, PATCH, OPTIONS',
      'Access-Control-Allow-Headers': 'X-Requested-With, content-type, Authorization'
    }
  },
  externals: {
    'react': 'React',
    'react-dom': 'ReactDOM'
  },
  plugins: [
    new webpack.DefinePlugin({
      'process.env.NODE_ENV': JSON.stringify('development')
    })
  ],
  stats: {
    colors: true,
    modules: false,
    reasons: false,
    errorDetails: true
  }
};